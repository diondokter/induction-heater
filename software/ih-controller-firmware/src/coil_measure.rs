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

    let mut adc_sample_buffer = [0u16; 128];

    // ----- Init the ADC -----
    let adc = pac::ADC1;

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

    loop {
        embassy_time::Timer::after(Duration::from_hz(
            modbus::COIL_MEASURE_FREQUENCY.read().await.clamp(1, 1000) as u64,
        ))
        .await;

        if !modbus::COIL_POWER_ENABLE.read().await {
            continue;
        }

        // Wait for the last conversion to be done
        while adc.cr().read().adstp() {
            embassy_futures::yield_now().await;
        }

        let dma_transfer = unsafe {
            embassy_stm32::dma::Transfer::new_read::<u16>(
                &mut measure_dma,
                5,                         // ADC request
                ADC1.dr().as_ptr().cast(), // The result register of the adc
                &mut adc_sample_buffer,
                TransferOptions::default(),
            )
        };

        adc.cr().modify(|reg| reg.set_adstart(true));

        dma_transfer.await;

        adc.cr().modify(|reg| reg.set_adstp(true));

        // Take the samples and do the calculations

        // Stats:
        // ADC clock = 64Mhz
        // ADC trigger delay = 3.5 clocks = 54.6875ns
        // ADC sample speed = 2.5Msps = 400ns
        // Time per sample = 1.5 clocks = 23.4375ns
        // We'll take the middle point of the sample period as THE time of the sample.
        //
        // Time of sample = index * 400ns + 54.6875ns + 23.4375ns/2 = index * 400ns + ~66ns
        //
        // Sample is 12-bit = 0..=4095
        // Vref = 3.3v
        //
        // Voltage of sample = sample * 3.3 / 4095.0 = sample *

        let nanos_volt_iter = adc_sample_buffer
            .iter()
            .enumerate()
            .map(|(index, &sample)| {
                let nanos = index as u32 * 400 + 66;
                let volts = sample as f32 * (3.3 / 4095.0);

                (nanos as f32 / 1000000000.0, volts)
            });

        // We want to be accurate with the frequency.
        // The voltage follows a sine wave and we want to fit one to the samples.
        // First we find the highest sample to get a good guess as to what the max voltage is.

        let (sin_peak_time_estimate, max_voltage_estimate) = nanos_volt_iter.clone().fold(
            (0.0, 0.0f32),
            |(acc_time, acc_voltage), (time, voltage)| {
                if voltage > acc_voltage {
                    (time, voltage)
                } else {
                    (acc_time, acc_voltage)
                }
            },
        );

        let initial_frequency_estimate = 1.0 / (sin_peak_time_estimate * 4.0);
        defmt::info!(
            "Initial frequency estimate: {} (@ peak {} volts)",
            initial_frequency_estimate,
            max_voltage_estimate
        );

        let initial_fit_error = calculate_fit_error(
            nanos_volt_iter.clone(),
            initial_frequency_estimate,
            max_voltage_estimate,
        );
        defmt::info!("Initial fit error: {}", initial_fit_error);

        // Let's assume the estimate is at most one sample off
        let max_frequency = 1.0 / ((sin_peak_time_estimate - 0.0000004) * 4.0);
        let min_frequency = 1.0 / ((sin_peak_time_estimate + 0.0000004) * 4.0);

        defmt::info!(
            "Frequency range {{ max: {}, min: {} }}",
            max_frequency,
            min_frequency
        );

        const NUM_FREQS_TEST: u8 = 32;
        let test_frequencies = (0..NUM_FREQS_TEST).map(|i| {
            min_frequency + i as f32 * (max_frequency - min_frequency) / (NUM_FREQS_TEST - 1) as f32
        });

        let fit_errors = test_frequencies.map(|freq| {
            (
                freq,
                calculate_fit_error(nanos_volt_iter.clone(), freq, max_voltage_estimate),
            )
        });

        let best_frequency = fit_errors.fold(
            (0.0f32, f32::INFINITY),
            |(acc_freq, acc_fit), (freq, fit)| {
                if fit < acc_fit {
                    (freq, fit)
                } else {
                    (acc_freq, acc_fit)
                }
            },
        );

        defmt::info!(
            "Best frequency: {} (@ fit {})",
            best_frequency.0,
            best_frequency.1
        );
    }
}

fn calculate_fit_error(
    sample_iter: impl Iterator<Item = (f32, f32)>,
    frequency: f32,
    max_value: f32,
) -> f32 {
    sample_iter
        .map(|(time, value)| {
            let sin_value = libm::sinf(time * frequency * core::f32::consts::TAU) * max_value;
            libm::fabsf(value - sin_value)
        })
        .sum()
}
