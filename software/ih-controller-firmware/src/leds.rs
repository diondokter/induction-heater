use core::pin::pin;

use crate::modbus;
use embassy_futures::select::{select, Either};
use embassy_stm32::{
    gpio::OutputOpenDrain,
    peripherals::{PA1, PA2},
};
use futures::StreamExt;

#[embassy_executor::task]
pub async fn leds(
    mut led_g: OutputOpenDrain<'static, PA2>,
    mut led_r: OutputOpenDrain<'static, PA1>,
) -> ! {
    let mut led_g_listener = pin!(modbus::LED_GREEN.get_listener().await);
    let mut led_r_listener = pin!(modbus::LED_RED.get_listener().await);

    modbus::LED_GREEN.write(false).await;
    modbus::LED_RED.write(false).await;

    loop {
        let result = select(led_g_listener.next(), led_r_listener.next()).await;

        match result {
            Either::First(Some(value)) => led_g.set_level((!value).into()),
            Either::Second(Some(value)) => led_r.set_level((!value).into()),
            _ => defmt::unreachable!(),
        }
    }
}
