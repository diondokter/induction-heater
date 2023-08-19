use embassy_stm32::gpio::low_level::{AFType, Pin};
use embassy_stm32::interrupt;
use embassy_stm32::peripherals::{PB9, TIM17};
use embassy_stm32::rcc::low_level::RccPeripheral;
use embassy_stm32::timer::low_level::Basic16bitInstance;
use embassy_stm32::timer::Channel1Pin;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant};

use crate::modbus;

#[embassy_executor::task]
pub async fn tacho_measure(_timer: TIM17, pin: PB9) -> ! {
    // Set the pin to the right AF mode
    pin.set_as_af(pin.af_num(), AFType::Input);

    const CLOCK_FREQ: f32 = 64_000_000.0;
    const PRESCALER: f32 = 128.0;
    const TIMER_FREQ: f32 = CLOCK_FREQ / PRESCALER;
    const TIMER_PERIOD: f32 = 1.0 / TIMER_FREQ;

    let regs = unsafe { &*stm32g0::stm32g030::TIM17::ptr() };
    // Make sure the clock runs at 64 MHz
    defmt::assert_eq!(TIM17::frequency().0, CLOCK_FREQ as u32);
    // Set the prescaler
    regs.psc.write(|w| w.psc().variant(PRESCALER as u16 - 1));
    // Wrap the least amount possible
    regs.arr.write(|w| w.arr().variant(0xFFFF));
    // No repetitions
    regs.rcr.write(|w| w.rep().variant(0));
    // Set input from CH1
    regs.tisel.modify(|_, w| w.ti1sel().variant(0));
    // Select the active input
    regs.ccr1.write(|w| w.ccr1().variant(0b01));
    while regs.ccr1.read().ccr1().bits() == 0 {}
    // Set the input filtering
    regs.ccmr1_input().modify(|_, w|
            // No filtering
            w.ic1f().variant(0)
            // No prescaler
            .ic1psc().variant(0));
    // Set the edge of active transition to rising edge and set the capture enabled
    regs.ccer
        .modify(|_, w| w.cc1p().clear_bit().cc1e().set_bit());
    // Set the interrupt
    regs.dier.modify(|_, w| w.cc1ie().set_bit().uie().enabled());
    // Start the timer
    regs.cr1.modify(|_, w| w.cen().enabled());

    let mut previous_capture: u32 = 0;
    let mut next_capture: u32 = 0;

    let mut last_capture_time = Instant::now();

    loop {
        let capture = TIM17_CH1_INPUT_CAPTURE.wait().await;
        let mut error = false;

        match capture {
            Event::Reload(value) => next_capture += value as u32,
            Event::Capture(value) => {
                next_capture += value as u32;

                let diff = next_capture - previous_capture;
                previous_capture = next_capture % 0x1_0000;

                let time_diff = diff as f32 * TIMER_PERIOD;
                // Calculate and divide by 2 because we get two pulses per rotation
                let tacho_frequency = 1.0 / time_diff / 2.0;
                let tacho_rpm = tacho_frequency * 60.0;

                modbus::FAN_RPM.write(tacho_rpm as u16).await;

                if tacho_rpm < 100.0 || tacho_rpm >= 10_000.0 {
                    error |= true;
                }

                last_capture_time = Instant::now();
            }
        }

        if last_capture_time.elapsed() > Duration::from_secs(1) {
            error |= true;
        }

        if modbus::LED_RED.read().await != error {
            modbus::LED_RED.write(error).await;
        }
    }
}

static TIM17_CH1_INPUT_CAPTURE: Signal<ThreadModeRawMutex, Event> = Signal::new();

/// Interrupt handler.
pub struct InterruptHandler {}

impl interrupt::typelevel::Handler<<TIM17 as Basic16bitInstance>::Interrupt> for InterruptHandler {
    unsafe fn on_interrupt() {
        let regs = unsafe { &*stm32g0::stm32g030::TIM17::ptr() };

        let sr = regs.sr.read();
        let compare_interrupt = sr.cc1if().bit_is_set();
        let update_interrupt = sr.uif().is_update_pending();

        if compare_interrupt {
            let capture_value = regs.ccr1.read().ccr1().bits();
            TIM17_CH1_INPUT_CAPTURE.signal(Event::Capture(capture_value));
        }

        if update_interrupt {
            // We've reloaded
            let reload_value = regs.arr.read().arr().bits();
            TIM17_CH1_INPUT_CAPTURE.signal(Event::Reload(reload_value));
        }

        // If we miss a capture, this interrupt gets active, but we'll just ignore it and turn it off
        // We also clear the update interrupt flag
        regs.sr.modify(|_, w| w.cc1of().set_bit().uif().clear());
    }
}

enum Event {
    /// Timer overflowed at the given point and reset to 0
    Reload(u16),
    /// Input edge was detected at this value
    Capture(u16),
}
