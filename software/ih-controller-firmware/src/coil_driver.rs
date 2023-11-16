use core::pin::pin;

use embassy_futures::select::{select3, Either3};
use embassy_stm32::{
    peripherals::TIM1,
    time::Hertz,
    timer::{simple_pwm::SimplePwm, Channel, OutputPolarity},
};
use embassy_time::Duration;
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

    const UPDATE_TICK_DURATION: Duration = Duration::from_millis(10);
    let mut update_ticker = embassy_time::Ticker::every(UPDATE_TICK_DURATION);

    let mut current_duty_cycle = 0.0f32;
    // The amount the dutycycle changes per second
    const DUTY_CYCLE_DELTA: f32 = 0.2;

    loop {
        let result = select3(
            update_ticker.next(),
            frequency_listener.next(),
            enable_listener.next(),
        )
        .await;

        match result {
            Either3::First(_) => {
                let target_dutycycle = if modbus::COIL_POWER_ENABLE.read() {
                    modbus::COIL_POWER_DUTYCYCLE.read()
                } else {
                    0.0
                };

                if target_dutycycle > current_duty_cycle {
                    let new_dutycycle = (current_duty_cycle
                        + DUTY_CYCLE_DELTA
                            * (UPDATE_TICK_DURATION.as_micros() as f32 / 1_000_000.0))
                        .min(target_dutycycle);

                    set_duty_cycle(&mut driver_pwm, new_dutycycle);

                    if current_duty_cycle >= f32::EPSILON {
                        driver_pwm.enable(Channel::Ch1);
                        driver_pwm.enable(Channel::Ch2);
                    }

                    current_duty_cycle = new_dutycycle;
                }

                if target_dutycycle < current_duty_cycle {
                    let new_dutycycle = (current_duty_cycle
                        - DUTY_CYCLE_DELTA
                            * (UPDATE_TICK_DURATION.as_micros() as f32 / 1_000_000.0))
                        .max(target_dutycycle);

                    set_duty_cycle(&mut driver_pwm, new_dutycycle);

                    if new_dutycycle <= f32::EPSILON {
                        driver_pwm.disable(Channel::Ch1);
                        driver_pwm.disable(Channel::Ch2);
                    }

                    current_duty_cycle = new_dutycycle;
                }
            }
            Either3::Second(Some(frequency)) => {
                driver_pwm.set_freq(Hertz(frequency.clamp(10_000, 80_000)));
                set_duty_cycle(&mut driver_pwm, current_duty_cycle);
            }
            Either3::Third(Some(enable)) => {
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
