use embassy_stm32::{
    dma::TransferOptions,
    pac::{self, rcc::vals::Ppre, ADC1},
    peripherals,
};
use embassy_time::Duration;

use crate::modbus;

#[embassy_executor::task]
pub async fn coil_measure(
    _measure_adc: peripherals::ADC1,
    _measure_pin: peripherals::PA0,
    mut measure_dma: peripherals::DMA1_CH3,
) -> ! {
    // What we need from the ADC is not implemented in embassy, so we'll have to control the registers ourselves.
    // We take the peripherals nonetheless so we know we've got exclusive access

    modbus::COIL_MEASURE_FREQUENCY.write(10).await;

    let mut adc_sample_buffer = [0u16; NUM_SAMPLES];
    let mut coil_sample_buffer = [CoilSample::EMPTY; NUM_SAMPLES];

    // ----- Init the ADC -----
    let adc = pac::ADC1;
    init_adc(&adc).await;

    loop {
        embassy_time::Timer::after(Duration::from_hz(
            modbus::COIL_MEASURE_FREQUENCY.read().await.clamp(1, 1000) as u64,
        ))
        .await;

        if !modbus::COIL_POWER_ENABLE.read().await {
            continue;
        }

        start_adc(&adc, &mut measure_dma, &mut adc_sample_buffer).await;

        // Take the samples and turn them into full coil samples
        for (index, (sample, coil_sample)) in adc_sample_buffer
            .iter()
            .zip(coil_sample_buffer.iter_mut())
            .enumerate()
        {
            *coil_sample = CoilSample::new(index, *sample);
        }

        // We want to be accurate with the frequency.
        // The voltage follows a sine wave and we want to fit one to the samples.
        // First we find the highest sample to get a good guess as to what the max voltage is.
        // This should be pretty accurate since the peak of the sine wave is pretty flat.
        let peak_sample = calculate_peak_sample(&coil_sample_buffer);

        // Let's assume the estimate is at most one sample off
        let mut max_freq = 1.0 / ((peak_sample.time - ADC_SAMPLE_PERIOD) * 4.0);
        let mut min_freq = 1.0 / ((peak_sample.time + ADC_SAMPLE_PERIOD) * 4.0);

        const NUM_FREQS_TEST: u32 = 11;
        const MAX_FREQ_ERROR: f32 = 5.0;

        while (max_freq - min_freq) > MAX_FREQ_ERROR {
            (min_freq, max_freq) = find_best_fit(
                &coil_sample_buffer,
                peak_sample,
                min_freq,
                max_freq,
                NUM_FREQS_TEST,
            );
        }

        let final_frequency = (min_freq + max_freq) / 2.0;
        defmt::info!("Final freq: {}", final_frequency);

        modbus::COIL_DRIVE_FREQUENCY
            .write(final_frequency as u32)
            .await;
        modbus::COIL_VOLTAGE_MAX.write(peak_sample.voltage).await;
    }
}

/// The amount of samples to take
const NUM_SAMPLES: usize = 64;

/// The clock speed of the ADC peripheral
const ADC_CLOCK: f32 = 64_000_000.0;
/// The time of one ADC clock tick
const ADC_CLOCK_TIME: f32 = 1.0 / ADC_CLOCK;
/// The amount of ADC clock tick delays after start event before the ADC starts
const ADC_TRIGGER_DELAY_CLOCKS: f32 = 3.5;
/// The amount of time between trigger and ADC start
const ADC_TRIGGER_DELAY_TIME: f32 = ADC_CLOCK_TIME * ADC_TRIGGER_DELAY_CLOCKS;
/// The sample speed of the ADC
const ADC_SAMPLE_SPEED: f32 = 2_500_000.0;
/// The time one sample takes
const ADC_SAMPLE_PERIOD: f32 = 1.0 / ADC_SAMPLE_SPEED;
/// The amount of ADC clocks used as the sampling time. The sample and hold will be held after these clocks.
const ADC_CLOCKS_PER_SAMPLE: f32 = 1.5;
/// The amount of time used as the sampling time
const ADC_SAMPLE_TIME: f32 = ADC_CLOCK_TIME * ADC_CLOCKS_PER_SAMPLE;
/// The time of a sample by index after the start of the trigger
fn time_of_sample(index: usize) -> f32 {
    ADC_TRIGGER_DELAY_TIME + index as f32 * ADC_SAMPLE_PERIOD + ADC_SAMPLE_TIME
}
/// The amount of bits the ADC samples with
const ADC_SAMPLE_BITS: u16 = 12;
/// The maximum value of a sample
const ADC_SAMPLE_MAX_VALUE: u16 = (1 << ADC_SAMPLE_BITS) - 1;
/// The reference voltage of the ADC (max sample value voltage)
const ADC_VREF: f32 = 3.3;
/// First resistor value (ohms) of the divider (connected to coil)
const VOLTAGE_DIVIDER_R1: f32 = 10_000.0;
/// Second resistor value (ohms) of the divider (connected to ground)
const VOLTAGE_DIVIDER_R2: f32 = 120.0;
/// Calculates the coil voltage based on the given sample
fn voltage_of_sample(sample: u16) -> f32 {
    let adc_voltage = sample as f32 * ADC_VREF / ADC_SAMPLE_MAX_VALUE as f32;
    adc_voltage * (VOLTAGE_DIVIDER_R1 + VOLTAGE_DIVIDER_R2) / VOLTAGE_DIVIDER_R2
}

fn calculate_peak_sample(samples: &[CoilSample; NUM_SAMPLES]) -> &CoilSample {
    samples
        .iter()
        .fold((&CoilSample::EMPTY, false), |(acc_sample, done), sample| {
            if sample.voltage > acc_sample.voltage && !done {
                (sample, done)
            } else if sample.voltage < acc_sample.voltage * 0.9 && !done {
                (acc_sample, true)
            } else {
                (acc_sample, done)
            }
        })
        .0
}

fn find_best_fit(
    samples: &[CoilSample; NUM_SAMPLES],
    peak_sample: &CoilSample,
    min_freq: f32,
    max_freq: f32,
    num_freqs_test: u32,
) -> (f32, f32) {
    defmt::info!("Frequency range {{ max: {}, min: {} }}", max_freq, min_freq);

    let freq_step = (max_freq - min_freq) / (num_freqs_test - 1) as f32;

    let test_frequencies = (0..num_freqs_test).map(|i| min_freq + i as f32 * freq_step);

    let mut best_frequency = 0.0;
    let mut best_fit = f32::INFINITY;

    for freq in test_frequencies {
        let fit = calculate_fit_error(samples, freq, peak_sample);
        defmt::info!("Freq: {}, fit: {}", freq, fit);

        if fit < best_fit {
            best_fit = fit;
            best_frequency = freq;
        } else {
            break;
        }
    }

    defmt::info!("Best frequency: {} (@ fit {})", best_frequency, best_fit);

    (
        best_frequency - freq_step * core::f32::consts::FRAC_1_SQRT_2,
        best_frequency + freq_step * core::f32::consts::FRAC_1_SQRT_2,
    )
}

fn calculate_fit_error(
    samples: &[CoilSample; NUM_SAMPLES],
    frequency: f32,
    peak_sample: &CoilSample,
) -> f32 {
    use micromath::F32Ext;

    samples
        .iter()
        .map(|sample| {
            let ideal_voltage = (sample.time * frequency * core::f32::consts::TAU)
                .sin()
                .max(0.0)
                * peak_sample.voltage;
            (sample.voltage - ideal_voltage).abs()
        })
        .sum()
}

#[derive(Clone, Copy, defmt::Format)]
struct CoilSample {
    time: f32,
    voltage: f32,
}

impl CoilSample {
    const EMPTY: Self = Self {
        time: 0.0,
        voltage: 0.0,
    };

    fn new(index: usize, sample: u16) -> Self {
        Self {
            time: time_of_sample(index),
            voltage: voltage_of_sample(sample),
        }
    }
}

async fn init_adc(adc: &embassy_stm32::pac::adc::Adc) {
    // Check if our system clocks are in order.
    // APB must not be divided for the adc max clock setting to work
    // while not having jitter from the timer trigger.
    defmt::assert!(
        pac::RCC.cfgr().read().ppre() == Ppre::DIV1,
        "Bad APB prescaler setting"
    );

    // Set the ADC clock to PCLK without divider.
    adc.cfgr2().modify(|reg| reg.set_ckmode(0b11));

    // Enable the ADC clock
    critical_section::with(|_| {
        pac::RCC.apbenr2().modify(|w| w.set_adcen(true));
    });

    // Turn on the voltage regulator. This is required to run the ADC
    adc.cr().modify(|w| w.set_advregen(true));
    adc.cfgr1().modify(|w| {
        // We're not gonna use the channel sequencer, so make sure it's off
        w.set_chselrmod(false);
        // We want to do multiple conversions, so go to continuous mode
        w.set_cont(true);
        // We trigger the ADC from the TIM1_TRGO2 event
        w.set_extsel(0b000);
        // We trigger the ADC on the rising edge (TODO this might need to be falling edge 0b10)
        w.set_exten(0b01);
    });
    // We need to wait
    embassy_time::Timer::after(Duration::from_micros(20)).await;
    // Start the ADC calibration. Technicall not required, but it make it more accurate
    adc.cr().modify(|reg| reg.set_adcal(true));
    // Wait for the calibration to be done
    while adc.cr().read().adcal() {
        embassy_futures::yield_now().await;
    }
    // More waiting
    embassy_time::Timer::after(Duration::from_micros(1)).await;
    adc.cfgr1().modify(|reg| {
        // Set the resolution of the conversion
        reg.set_res(pac::adc::vals::Res::TWELVEBIT);
        // We want the ADC to trigger DMA on conversion
        reg.set_dmaen(true);
        // The DMA is in fixed length mode, not circular mode
        reg.set_dmacfg(false);
    });
    // Make sure bits are off
    while adc.cr().read().addis() {
        embassy_futures::yield_now().await;
    }
    // Enable ADC
    adc.isr().modify(|reg| {
        reg.set_adrdy(true);
    });
    adc.cr().modify(|reg| {
        reg.set_aden(true);
    });
    while !adc.isr().read().adrdy() {
        embassy_futures::yield_now().await;
    }
    // Set the sample time
    adc.smpr()
        .modify(|reg| reg.set_smp1(pac::adc::vals::SampleTime::CYCLES1_5));
    // Select channel. PA0 is channel 0
    adc.chselr().write(|reg| reg.set_chsel(1 << 0));
    adc.isr().modify(|reg| {
        reg.set_eos(true);
    });
}

async fn start_adc(
    adc: &embassy_stm32::pac::adc::Adc,
    measure_dma: &mut peripherals::DMA1_CH3,
    sample_buffer: &mut [u16],
) {
    // Wait for the last conversion to be done
    while adc.cr().read().adstp() {
        embassy_futures::yield_now().await;
    }

    let dma_transfer = unsafe {
        embassy_stm32::dma::Transfer::new_read::<u16>(
            measure_dma,
            5,                         // ADC request
            ADC1.dr().as_ptr().cast(), // The result register of the adc
            sample_buffer,
            TransferOptions::default(),
        )
    };

    adc.cr().modify(|reg| reg.set_adstart(true));

    dma_transfer.await;

    adc.cr().modify(|reg| reg.set_adstp(true));
}
