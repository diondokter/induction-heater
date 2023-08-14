use embassy_stm32::{dma::TransferOptions, pac::ADC1, peripherals};
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

    let mut adc_sample_buffer = [0u16; 64];

    // TODO: Configure the ADC and the pin

    loop {
        embassy_time::Timer::after(Duration::from_hz(
            modbus::COIL_MEASURE_FREQUENCY.read().await.clamp(1, 1000) as u64,
        ))
        .await;

        if !modbus::COIL_POWER_ENABLE.read().await {
            continue;
        }

        // TODO: Make the ADC start on the next TRGO2 (either rising edge or falling edge, needs figuring out)

        let dma_transfer = unsafe {
            embassy_stm32::dma::Transfer::new_read::<u16>(
                &mut measure_dma,
                5,                         // ADC request
                ADC1.dr().as_ptr().cast(), // The result register of the adc
                &mut adc_sample_buffer,
                TransferOptions::default(),
            )
        };

        dma_transfer.await;

        // TODO: Do ADC cleanup
        // TODO: Take the samples and do the calculations
    }
}
