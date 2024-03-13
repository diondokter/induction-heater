use core::pin::pin;

use embassy_futures::select::{select, Either};
use embassy_stm32::{
    peripherals::TIM1,
    time::Hertz,
    timer::{simple_pwm::SimplePwm, Channel, OutputPolarity},
};
use futures::StreamExt;

use crate::modbus;

#[embassy_executor::task]
pub async fn coil_driver(mut driver_pwm: SimplePwm<'static, TIM1>) -> ! {
    let mut enable_listener = pin!(modbus::COIL_POWER_ENABLE.get_listener().await);
    let mut frequency_listener = pin!(modbus::COIL_DRIVE_FREQUENCY.get_listener().await);

    // We want the timer to be able to trigger the ADC.
    // For that we set the MMS2 to `0101: Compare - OC2REFC signal is used as trigger output (TRGO2)`
    // The adc can then trigger on the TRGO2 signal
    unsafe { &*stm32g0::stm32g030::TIM1::PTR }
        .cr2
        .modify(|_, w| w.mms2().variant(0b0101));

    driver_pwm.set_polarity(Channel::Ch1, OutputPolarity::ActiveLow);

    modbus::COIL_POWER_ENABLE.write(false);
    modbus::COIL_DRIVE_FREQUENCY.write(40_000);
    modbus::COIL_POWER_DUTYCYCLE.write(0.5);

    loop {
        let result = select(frequency_listener.next(), enable_listener.next()).await;

        match result {
            Either::First(Some(frequency)) => {
                driver_pwm.set_freq(Hertz(frequency.clamp(10_000, 80_000)));
                set_duty_cycle(&mut driver_pwm, modbus::COIL_POWER_DUTYCYCLE.read());
            }
            Either::Second(Some(enable)) => {
                modbus::LED_GREEN.write(enable);
            }
            _ => defmt::unreachable!(),
        }
    }
}

fn set_duty_cycle(driver_pwm: &mut SimplePwm<'static, TIM1>, dutycycle: f32) {
    let max_duty = driver_pwm.get_max_duty();
    let duty_value = (max_duty as f32 * dutycycle.clamp(0.0, 1.0)) as u16;

    driver_pwm.set_duty(Channel::Ch1, duty_value);
    driver_pwm.set_duty(Channel::Ch2, max_duty - duty_value);
}
