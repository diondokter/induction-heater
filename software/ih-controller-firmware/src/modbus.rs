use super::USART1;
use embassy_stm32::peripherals::{DMA1_CH1, DMA1_CH2};
use embassy_stm32::usart::Uart;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use rmodbus::server::context::ModbusContext;
use rmodbus::server::ModbusFrame;
use rmodbus::ModbusFrameBuf;

#[embassy_executor::task]
pub(crate) async fn modbus_server(
    unit_id: u8,
    mut rs485: Uart<'static, USART1, DMA1_CH1, DMA1_CH2>,
    modbus_context: &'static Mutex<ThreadModeRawMutex, ModbusContext<0, 2, 8, 8>>,
) -> ! {
    let mut receive_buffer: ModbusFrameBuf = [0; 256];
    let mut response_buffer = heapless::Vec::<_, 256>::new();

    loop {
        defmt::trace!("Reading modbus uart");
        if let Err(e) = rs485.read_until_idle(&mut receive_buffer).await {
            defmt::error!("Could not read rs485 bus: {}", e);
            continue;
        }
        defmt::trace!("Received modbus frame");

        let mut frame = ModbusFrame::new(
            unit_id,
            &receive_buffer,
            rmodbus::ModbusProto::Rtu,
            &mut response_buffer,
        );

        if let Err(e) = frame.parse() {
            defmt::error!("Could not parse modbus frame: {}", e as u8);
            continue;
        }

        if frame.processing_required {
            let result = match frame.readonly {
                true => frame.process_read(&*modbus_context.lock().await),
                false => frame.process_write(&mut *modbus_context.lock().await),
            };
            if let Err(e) = result {
                defmt::error!("Could not process modbus frame: {}", e as u8);
                continue;
            }
        }
        if frame.response_required {
            if let Err(e) = frame.finalize_response() {
                defmt::error!("Could not finalize modbus frame: {}", e as u8);
                continue;
            }

            defmt::trace!("Sending modbus response");

            if let Err(e) = rs485.write(&response_buffer).await {
                defmt::error!("Could not send modbus frame response: {}", e);
            }
        }
    }
}
