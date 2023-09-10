#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(generic_arg_infer)]

use embassy_executor::{InterruptExecutor, SendSpawner, Spawner};
use embassy_stm32::{
    bind_interrupts,
    gpio::{Level, OutputOpenDrain, OutputType, Pull, Speed},
    interrupt::{self, InterruptExt, Priority},
    peripherals::{self, USART1},
    rcc::{AHBPrescaler, APBPrescaler, ClockSrc, PllConfig},
    time::khz,
    timer::{
        complementary_pwm::{ComplementaryPwm, ComplementaryPwmPin},
        simple_pwm::PwmPin,
    },
    usart::{self, Uart},
};
use {defmt_rtt as _, panic_probe as _};

mod coil_driver;
mod coil_measure;
mod leds;
pub mod modbus;
mod tacho_measure;

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

static EXECUTOR_HIGH: InterruptExecutor = InterruptExecutor::new();

#[cortex_m_rt::interrupt]
unsafe fn RTC_TAMP() {
    EXECUTOR_HIGH.on_interrupt()
}

#[embassy_executor::main]
async fn main(thread_mode_spawner: Spawner) {
    interrupt::RTC_TAMP.set_priority(Priority::P15);
    let interrupt_spawner = EXECUTOR_HIGH.start(interrupt::RTC_TAMP);

    run(thread_mode_spawner, interrupt_spawner).await;
}

async fn run(thread_mode_spawner: Spawner, interrupt_spawner: SendSpawner) {
    let mut stm_config = embassy_stm32::Config::default();
    stm_config.rcc.mux = ClockSrc::PLL(PllConfig::default()); // Make the core run at 64 Mhz
    stm_config.rcc.ahb_pre = AHBPrescaler::NotDivided; // We want everything to run at 64 Mhz
    stm_config.rcc.apb_pre = APBPrescaler::NotDivided; // Required for the ADC (and we want everything to run at 64 Mhz)
    let p = embassy_stm32::init(stm_config);

    // Remap PA11 to PA9. All other IO's are fine
    embassy_stm32::pac::SYSCFG
        .cfgr1()
        .modify(|w| w.set_pa11_rmp(true));

    let measure_adc = p.ADC1;
    let measure_pin = p.PA0; // ADC_IN0
    let measure_dma = p.DMA1_CH3;

    let led_g = OutputOpenDrain::new(p.PA2, Level::High, Speed::Low, Pull::None);
    let led_r = OutputOpenDrain::new(p.PA1, Level::High, Speed::Low, Pull::None);

    let mut config = usart::Config::default();
    config.baudrate = 921600;
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

    let fan_tacho_timer = p.TIM17;
    let fan_tacho_pin = p.PB9; // CH1

    thread_mode_spawner.must_spawn(leds::leds(led_g, led_r));
    interrupt_spawner.must_spawn(modbus::modbus_server(1, rs485));
    interrupt_spawner.must_spawn(coil_driver::coil_driver(driver_pwm));
    thread_mode_spawner.must_spawn(coil_measure::coil_measure(
        measure_adc,
        measure_pin,
        measure_dma,
    ));
    thread_mode_spawner.must_spawn(tacho_measure::tacho_measure(fan_tacho_timer, fan_tacho_pin));
}
