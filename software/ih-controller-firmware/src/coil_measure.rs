use defmt::unwrap;
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
    defmt::debug!("ADC sampling speed is: {}", ADC_SAMPLE_SPEED);

    // What we need from the ADC is not implemented in embassy, so we'll have to control the registers ourselves.
    // We take the peripherals nonetheless so we know we've got exclusive access

    let mut adc_sample_buffer = [0u16; NUM_SAMPLES];
    let mut coil_sample_buffer = [CoilSample::EMPTY; NUM_SAMPLES];

    // ----- Init the ADC -----
    let adc = pac::ADC1;
    init_adc(&adc).await;

    let mut ticker = embassy_time::Ticker::every(Duration::from_millis(50));

    loop {
        ticker.next().await;

        if !modbus::COIL_POWER_ENABLE.read() {
            continue;
        }

        start_adc(&adc, &mut measure_dma, &mut adc_sample_buffer).await;

        modbus::ADC_SAMPLES.write(adc_sample_buffer);

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
        let peak = calculate_peak_sample(&coil_sample_buffer);

        let peak_sample = &coil_sample_buffer[peak.top];

        // defmt::info!("Peak: {}, top sample: {}", peak, peak_sample);
        // defmt::info!("{}", coil_sample_buffer);

        // We only use the range of the peak
        let sample_range =
            peak.start.unwrap_or(0)..=peak.end.unwrap_or(coil_sample_buffer.len() - 1);
        let coil_sample_buffer_peak = &coil_sample_buffer[sample_range];

        // Let's assume the estimate is at most one sample off
        let mut max_freq = 1.0 / ((peak_sample.time - ADC_SAMPLE_PERIOD) * 4.0);
        let mut min_freq = 1.0 / ((peak_sample.time + ADC_SAMPLE_PERIOD) * 4.0);

        const NUM_FREQS_TEST: u32 = 11;
        const MAX_FREQ_ERROR: f32 = 10.0;

        let mut i = 0;

        while (max_freq - min_freq) > MAX_FREQ_ERROR {
            if i > 20 {
                defmt::warn!(
                    "Max iteration reached and freq error is {}",
                    (max_freq - min_freq)
                );
                break;
            }

            (min_freq, max_freq) = find_best_fit(
                &coil_sample_buffer_peak,
                peak_sample,
                min_freq,
                max_freq,
                NUM_FREQS_TEST,
            );

            i += 1;
        }

        let mut final_frequency = (min_freq + max_freq) / 2.0;
        // defmt::info!("Final freq: {}", final_frequency);

        final_frequency = final_frequency.clamp(1_000.0, 100_000.0);

        let current_frequency = modbus::COIL_DRIVE_FREQUENCY.read() as f32;

        modbus::COIL_DRIVE_FREQUENCY
            .write((current_frequency * 0.98 + final_frequency * 0.02) as u32);
        modbus::COIL_VOLTAGE_MAX.write(peak_sample.voltage);
    }
}

/// The amount of samples to take
pub const NUM_SAMPLES: usize = 64;

/// The clock speed of the ADC peripheral
const ADC_CLOCK: f32 = 32_000_000.0;
/// The time of one ADC clock tick
const ADC_CLOCK_TIME: f32 = 1.0 / ADC_CLOCK;
/// The amount of ADC clock tick delays after start event before the ADC starts
const ADC_TRIGGER_DELAY_CLOCKS: f32 = 3.25;
/// The amount of time between trigger and ADC start
const ADC_TRIGGER_DELAY_TIME: f32 = ADC_CLOCK_TIME * ADC_TRIGGER_DELAY_CLOCKS;
/// The amount of ADC clocks used as the sampling time. The sample and hold will be held after these clocks.
const ADC_CLOCKS_PER_SAMPLE: f32 = 1.5;
/// The amount of time used as the sampling time
const ADC_SAMPLE_TIME: f32 = ADC_CLOCK_TIME * ADC_CLOCKS_PER_SAMPLE;
/// The amount of ADC clocks used for the succesive approximation. (Sample bits + 0.5)
const ADC_CLOCKS_PER_SAR: f32 = 12.5;
/// The amount of time used as the SAR time
const ADC_SAR_TIME: f32 = ADC_CLOCK_TIME * ADC_CLOCKS_PER_SAR;
/// The time one sample takes
const ADC_SAMPLE_PERIOD: f32 = ADC_SAMPLE_TIME + ADC_SAR_TIME;
/// The sample speed of the ADC
const ADC_SAMPLE_SPEED: f32 = 1.0 / ADC_SAMPLE_PERIOD;
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

fn calculate_peak_sample(samples: &[CoilSample; NUM_SAMPLES]) -> Peak {
    let average_voltage =
        samples.iter().map(|sample| sample.voltage).sum::<f32>() / samples.len() as f32;

    // We are going to define a peak as a collection of samples that sticks above the average.
    // The peak we want to return is the peak that is the widest, not necessarily the highest.
    // We operate in with fixed memory, so we may not be able to track all the peaks, but we can track the widest peak.
    // If a peak has one side not part of the samples,
    // then its width is counted as if it was there symmetrical with the other side of the peak.

    let mut peaks = heapless::Vec::<Peak, 4>::new();

    let mut current_peak: Option<Peak> = None;

    for (index, sample) in samples.iter().enumerate() {
        match &mut current_peak {
            Some(peak) if sample.voltage >= average_voltage => {
                if sample.voltage > samples[peak.top].voltage {
                    peak.top = index;
                }
            }
            Some(peak) => {
                peak.end = Some(index - 1);

                if peaks.is_full() {
                    let min_peak = unwrap!(peaks.iter_mut().min());
                    if peak > min_peak {
                        *min_peak = *peak;
                    }
                } else {
                    unwrap!(peaks.push(*peak));
                }

                current_peak = None;
            }
            None if sample.voltage >= average_voltage => {
                current_peak = Some(Peak {
                    top: index,
                    start: if index == 0 { None } else { Some(index) },
                    end: None,
                })
            }
            None => {}
        }
    }

    if let Some(peak) = current_peak {
        if peaks.is_full() {
            let min_peak = unwrap!(peaks.iter_mut().min());
            if peak > *min_peak {
                *min_peak = peak;
            }
        } else {
            unwrap!(peaks.push(peak));
        }
    }

    *unwrap!(peaks.iter().max())
}

#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
struct Peak {
    top: usize,
    start: Option<usize>,
    end: Option<usize>,
}

impl Peak {
    fn width(&self) -> usize {
        match (self.start, self.end) {
            (None, None) => 0, // Should never happen... but it is constructable
            (None, Some(end)) => end.saturating_sub(self.top) * 2,
            (Some(start), None) => self.top.saturating_sub(start) * 2,
            (Some(start), Some(end)) => end.saturating_sub(start),
        }
    }
}

impl PartialOrd for Peak {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Peak {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.width().cmp(&other.width())
    }
}

fn find_best_fit(
    samples: &[CoilSample],
    peak_sample: &CoilSample,
    min_freq: f32,
    max_freq: f32,
    num_freqs_test: u32,
) -> (f32, f32) {
    defmt::trace!("Frequency range {{ max: {}, min: {} }}", max_freq, min_freq);

    let freq_step = (max_freq - min_freq) / (num_freqs_test - 1) as f32;

    let test_frequencies = (0..num_freqs_test).map(|i| min_freq + i as f32 * freq_step);

    let mut best_frequency = 0.0;
    let mut best_fit = f32::INFINITY;

    for freq in test_frequencies {
        let fit = calculate_fit_error(samples, freq, peak_sample);
        defmt::trace!("Freq: {}, fit: {}", freq, fit);

        if fit < best_fit {
            best_fit = fit;
            best_frequency = freq;
        } else {
            break;
        }
    }

    defmt::trace!("Best frequency: {} (@ fit {})", best_frequency, best_fit);

    (
        best_frequency - freq_step * core::f32::consts::FRAC_1_SQRT_2,
        best_frequency + freq_step * core::f32::consts::FRAC_1_SQRT_2,
    )
}

fn calculate_fit_error(samples: &[CoilSample], frequency: f32, peak_sample: &CoilSample) -> f32 {
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

    // Enable the ADC clock
    critical_section::with(|_| {
        pac::RCC.apbenr2().modify(|w| w.set_adcen(true));
    });

    embassy_time::Timer::after(Duration::from_micros(1)).await;

    // Set the ADC clock to PCLK/2. (Clock max is 35Mhz, PCLK/2 is 32Mhz)
    adc.cfgr2().modify(|reg| reg.set_ckmode(0b01));

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
        w.set_exten(0b10);
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
