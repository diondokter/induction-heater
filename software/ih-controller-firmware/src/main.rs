#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Spawner;
use embassy_stm32::{
    bind_interrupts,
    gpio::{Level, OutputOpenDrain, OutputType, Pull, Speed},
    peripherals::{self, USART1},
    rcc::{ClockSrc, PllConfig},
    time::khz,
    timer::{
        complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin},
        simple_pwm::PwmPin,
    },
    usart::{self, Uart},
};
use {defmt_rtt as _, panic_probe as _};

mod driver;
mod leds;
mod measure;
pub mod modbus;

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    run(spawner).await;
}

async fn run(spawner: Spawner) {
    let mut stm_config = embassy_stm32::Config::default();
    stm_config.rcc.mux = ClockSrc::PLL(PllConfig::default()); // Make the core run at 64 Mhz
    let p = embassy_stm32::init(Default::default());

    // Remap PA11 to PA9. All other IO's are fine
    embassy_stm32::pac::SYSCFG
        .cfgr1()
        .modify(|w| w.set_pa11_rmp(true));

    let measure_adc = p.ADC1;
    let measure_pin = p.PA0; // ADC_IN0
    let measure_dma = p.DMA1_CH3;

    let led_g = OutputOpenDrain::new(p.PA2, Level::High, Speed::Low, Pull::None);
    let led_r = OutputOpenDrain::new(p.PA1, Level::High, Speed::Low, Pull::None);

    let config = usart::Config::default();
    let rs485 = Uart::new_with_de(
        p.USART1, p.PB7, p.PA9, Irqs, p.PA12, p.DMA1_CH1, p.DMA1_CH2, config,
    );

    let driver_pwm = ComplementaryPwm::new(
        p.TIM1,
        None,
        None,
        Some(PwmPin::new_ch2(p.PB3, OutputType::PushPull)),
        Some(ComplementaryPwmPin::new_ch2(p.PB0, OutputType::PushPull)),
        None,
        None,
        None,
        None,
        khz(40),
    );

    let _fan_tacho_timer = p.TIM17;
    let _fan_tacho = p.PB9; // CH1
                            // TODO input capture for fan tacho

    spawner.must_spawn(leds::leds(led_g, led_r));
    spawner.must_spawn(modbus::modbus_server(0, rs485));
    spawner.must_spawn(driver::driver(driver_pwm));
    spawner.must_spawn(measure::measure(measure_adc, measure_pin, measure_dma));
}
