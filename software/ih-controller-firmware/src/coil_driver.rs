use core::pin::pin;

use embassy_futures::select::{select3, Either3};
use embassy_stm32::{
    peripherals::TIM1,
    time::Hertz,
    timer::{complementary_pwm::ComplementaryPwm, Channel},
};
use futures::StreamExt;

use crate::modbus;

#[embassy_executor::task]
pub async fn coil_driver(mut driver_pwm: ComplementaryPwm<'static, TIM1>) -> ! {
    let mut enable_listener = pin!(modbus::COIL_POWER_ENABLE.get_listener().await);
    let mut frequency_listener = pin!(modbus::COIL_DRIVE_FREQUENCY.get_listener().await);
    let mut dutycycle_listener = pin!(modbus::COIL_POWER_DUTYCYCLE.get_listener().await);

    // We want the timer to be able to trigger the ADC.
    // For that we set the MMS2 to `0101: Compare - OC2REFC signal is used as trigger output (TRGO2)`
    // The adc can then trigger on the TRGO2 signal
    unsafe { &*stm32g0::stm32g030::TIM1::PTR }
        .cr2
        .modify(|_, w| w.mms2().variant(0b0101));

    driver_pwm.set_dead_time(0);

    modbus::COIL_POWER_ENABLE.write(false);
    modbus::COIL_DRIVE_FREQUENCY.write(38_000);
    modbus::COIL_POWER_DUTYCYCLE.write(0.5);

    loop {
        let result = select3(
            enable_listener.next(),
            frequency_listener.next(),
            dutycycle_listener.next(),
        )
        .await;

        match result {
            Either3::First(Some(enable)) => {
                if enable {
                    driver_pwm.enable(Channel::Ch2);
                } else {
                    driver_pwm.disable(Channel::Ch2);
                }

                modbus::LED_GREEN.write(enable);
            }
            Either3::Second(Some(frequency)) => {
                driver_pwm.set_freq(Hertz(frequency.max(1000)));
                let dutycycle = modbus::COIL_POWER_DUTYCYCLE.read();
                driver_pwm.set_duty(
                    Channel::Ch2,
                    (driver_pwm.get_max_duty() as f32 * dutycycle.clamp(0.0, 1.0)) as u16,
                );
            }
            Either3::Third(Some(dutycycle)) => {
                driver_pwm.set_duty(
                    Channel::Ch2,
                    (driver_pwm.get_max_duty() as f32 * dutycycle.clamp(0.0, 1.0)) as u16,
                );
            }
            _ => defmt::unreachable!(),
        }
    }
}
