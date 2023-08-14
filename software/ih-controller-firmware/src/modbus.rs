use core::future::poll_fn;
use core::marker::PhantomData;
use core::ops::RangeInclusive;
use core::task::{Context, Poll, Waker};

use super::USART1;
use defmt::unwrap;
use embassy_stm32::peripherals::{DMA1_CH1, DMA1_CH2};
use embassy_stm32::usart::Uart;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use futures::{Stream, StreamExt};
use rmodbus::consts::*;
use rmodbus::server::context::ModbusContext;
use rmodbus::server::ModbusFrame;
use rmodbus::ModbusFrameBuf;

static MODBUS_CONTEXT: Mutex<ThreadModeRawMutex, ModbusContext<8, 8, 8, 8>> =
    Mutex::<ThreadModeRawMutex, _>::new(ModbusContext::new());

static MODBUS_LISTENER_REGISTRATIONS: Mutex<
    ThreadModeRawMutex,
    [Option<ModbusListenerRegistration>; 8],
> = Mutex::new([REGINIT; 8]);
const REGINIT: Option<ModbusListenerRegistration> = None;

#[embassy_executor::task]
pub(crate) async fn modbus_server(
    unit_id: u8,
    mut rs485: Uart<'static, USART1, DMA1_CH1, DMA1_CH2>,
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
                true => frame.process_read(&*MODBUS_CONTEXT.lock().await),
                false => frame.process_write(&mut *MODBUS_CONTEXT.lock().await),
            };
            if let Err(e) = result {
                defmt::error!("Could not process modbus frame: {}", e as u8);
                continue;
            }

            let address_range = frame.reg..=frame.reg + frame.count;
            let address_type = match frame.func {
                MODBUS_GET_COILS | MODBUS_SET_COIL | MODBUS_SET_COILS_BULK => AddressType::Coil,
                MODBUS_GET_DISCRETES => AddressType::Discrete,
                MODBUS_GET_HOLDINGS | MODBUS_SET_HOLDING | MODBUS_SET_HOLDINGS_BULK => {
                    AddressType::Holding
                }
                MODBUS_GET_INPUTS => AddressType::Input,
                _ => {
                    defmt::unreachable!()
                }
            };

            if !frame.readonly {
                trigger_registrations(address_range, address_type).await;
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

async fn trigger_registrations(address_range: RangeInclusive<u16>, address_type: AddressType) {
    MODBUS_LISTENER_REGISTRATIONS
        .lock()
        .await
        .iter_mut()
        .flatten()
        .for_each(|registration| {
            registration.on_context_written(address_range.clone(), address_type)
        });
}

struct ModbusListenerRegistration {
    address_range: RangeInclusive<u16>,
    address_type: AddressType,
    waker: Option<Waker>,
}

impl ModbusListenerRegistration {
    fn new(
        address_range: RangeInclusive<u16>,
        address_type: AddressType,
        waker: Option<Waker>,
    ) -> Self {
        Self {
            address_range,
            address_type,
            waker,
        }
    }

    fn on_context_written(
        &mut self,
        address_range: RangeInclusive<u16>,
        address_type: AddressType,
    ) {
        if self.address_type == address_type
            && ranges_overlap(address_range, self.address_range.clone())
        {
            if let Some(waker) = self.waker.take() {
                waker.wake();
            }
        }
    }
}

struct ModbusListener {
    index: usize,
}

impl ModbusListener {
    async fn new(address_range: RangeInclusive<u16>, address_type: AddressType) -> Self {
        let waker = poll_fn(|cx| Poll::Ready(cx.waker().clone())).await;
        let mut registrations = MODBUS_LISTENER_REGISTRATIONS.lock().await;

        let index = unwrap!(registrations
            .iter()
            .enumerate()
            .find(|(_, r)| r.is_none())
            .map(|(i, _)| i));

        registrations[index] = Some(ModbusListenerRegistration::new(
            address_range,
            address_type,
            Some(waker),
        ));

        Self { index }
    }

    fn associate_bool<A: BitAddress + 'static>(
        self,
        register: &ModbusRegister<bool, A>,
    ) -> impl Stream<Item = bool> + '_ {
        self.then(|_| async { register.read().await })
    }

    fn associate_u16<A: RegisterAddress + 'static>(
        self,
        register: &ModbusRegister<u16, A>,
    ) -> impl Stream<Item = u16> + '_ {
        self.then(|_| async { register.read().await })
    }

    fn associate_u32<A: RegisterAddress + 'static>(
        self,
        register: &ModbusRegister<u32, A>,
    ) -> impl Stream<Item = u32> + '_ {
        self.then(|_| async { register.read().await })
    }

    fn associate_f32<A: RegisterAddress + 'static>(
        self,
        register: &ModbusRegister<f32, A>,
    ) -> impl Stream<Item = f32> + '_ {
        self.then(|_| async { register.read().await })
    }
}

impl futures::Stream for ModbusListener {
    type Item = ();

    fn poll_next(
        self: core::pin::Pin<&mut Self>,
        cx: &mut Context<'_>,
    ) -> Poll<Option<Self::Item>> {
        // Should work because this is a private threadmode mutex
        let mut registrations = unwrap!(MODBUS_LISTENER_REGISTRATIONS.try_lock());

        let waker = &mut unwrap!(registrations[self.index].as_mut()).waker;

        if waker.is_none() {
            *waker = Some(cx.waker().clone());
            Poll::Ready(Some(()))
        } else {
            Poll::Pending
        }
    }
}

impl Drop for ModbusListener {
    fn drop(&mut self) {
        // Should work because this is a private threadmode mutex
        unwrap!(MODBUS_LISTENER_REGISTRATIONS.try_lock())[self.index] = None;
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum AddressType {
    Coil,
    Discrete,
    Input,
    Holding,
}

fn ranges_overlap(x: RangeInclusive<u16>, y: RangeInclusive<u16>) -> bool {
    x.start() <= y.end() && y.start() <= x.end()
}

// ----- Control (holdings) -----
/// The amount of times per second the heater coil resonance frequency and voltage max is measured
pub const COIL_MEASURE_FREQUENCY: ModbusRegister<u16, Holdings> = ModbusRegister::new(0);
/// The dutycycle of the PWM that drives the coil. 0..=1
pub const COIL_POWER_DUTYCYCLE: ModbusRegister<f32, Holdings> = ModbusRegister::new(1);
// ----- Control (coils) -----
pub const COIL_POWER_ENABLE: ModbusRegister<bool, Coils> = ModbusRegister::new(0);

// ----- Report (inputs) -----
pub const COIL_DRIVE_FREQUENCY: ModbusRegister<u32, Inputs> = ModbusRegister::new(0);
pub const COIL_VOLTAGE_MAX: ModbusRegister<f32, Inputs> = ModbusRegister::new(2);
pub const FAN_RPM: ModbusRegister<u16, Inputs> = ModbusRegister::new(4);
// ----- Report (discretes) -----
pub const LED_GREEN: ModbusRegister<bool, Discretes> = ModbusRegister::new(0);
pub const LED_RED: ModbusRegister<bool, Discretes> = ModbusRegister::new(1);

pub struct ModbusRegister<T, A> {
    pub address_start: u16,
    _phantom: PhantomData<(T, A)>,
}

impl<T, A> ModbusRegister<T, A> {
    pub const fn new(address_start: u16) -> Self {
        Self {
            address_start,
            _phantom: PhantomData,
        }
    }
}

impl<A: BitAddress> ModbusRegister<bool, A> {
    pub async fn read(&self) -> bool {
        match A::TYPE {
            AddressType::Coil => unwrap!(MODBUS_CONTEXT
                .lock()
                .await
                .get_coil(self.address_start)
                .map_err(|e| e as u8)),
            AddressType::Discrete => unwrap!(MODBUS_CONTEXT
                .lock()
                .await
                .get_discrete(self.address_start)
                .map_err(|e| e as u8)),
            _ => defmt::unreachable!(),
        }
    }

    pub async fn write(&self, value: bool) {
        match A::TYPE {
            AddressType::Coil => unwrap!(MODBUS_CONTEXT
                .lock()
                .await
                .set_coil(self.address_start, value)
                .map_err(|e| e as u8)),
            AddressType::Discrete => unwrap!(MODBUS_CONTEXT
                .lock()
                .await
                .set_discrete(self.address_start, value)
                .map_err(|e| e as u8)),
            _ => defmt::unreachable!(),
        };

        trigger_registrations(self.address_start..=self.address_start, A::TYPE).await;
    }

    pub async fn get_listener(&self) -> impl Stream<Item = bool> + '_ {
        ModbusListener::new(self.address_start..=self.address_start, A::TYPE)
            .await
            .associate_bool::<A>(self)
    }
}

impl<A: RegisterAddress> ModbusRegister<f32, A> {
    pub async fn read(&self) -> f32 {
        match A::TYPE {
            AddressType::Input => unwrap!(MODBUS_CONTEXT
                .lock()
                .await
                .get_inputs_as_f32(self.address_start)
                .map_err(|e| e as u8)),
            AddressType::Holding => unwrap!(MODBUS_CONTEXT
                .lock()
                .await
                .get_holdings_as_f32(self.address_start)
                .map_err(|e| e as u8)),
            _ => defmt::unreachable!(),
        }
    }

    pub async fn write(&self, value: f32) {
        match A::TYPE {
            AddressType::Input => unwrap!(MODBUS_CONTEXT
                .lock()
                .await
                .set_inputs_from_f32(self.address_start, value)
                .map_err(|e| e as u8)),
            AddressType::Holding => unwrap!(MODBUS_CONTEXT
                .lock()
                .await
                .set_holdings_from_f32(self.address_start, value)
                .map_err(|e| e as u8)),
            _ => defmt::unreachable!(),
        };

        trigger_registrations(self.address_start..=self.address_start + 1, A::TYPE).await;
    }

    pub async fn get_listener(&self) -> impl Stream<Item = f32> + '_ {
        ModbusListener::new(self.address_start..=self.address_start + 1, A::TYPE)
            .await
            .associate_f32(self)
    }
}

impl<A: RegisterAddress> ModbusRegister<u32, A> {
    pub async fn read(&self) -> u32 {
        match A::TYPE {
            AddressType::Input => unwrap!(MODBUS_CONTEXT
                .lock()
                .await
                .get_inputs_as_u32(self.address_start)
                .map_err(|e| e as u8)),
            AddressType::Holding => unwrap!(MODBUS_CONTEXT
                .lock()
                .await
                .get_holdings_as_u32(self.address_start)
                .map_err(|e| e as u8)),
            _ => defmt::unreachable!(),
        }
    }

    pub async fn write(&self, value: u32) {
        match A::TYPE {
            AddressType::Input => unwrap!(MODBUS_CONTEXT
                .lock()
                .await
                .set_inputs_from_u32(self.address_start, value)
                .map_err(|e| e as u8)),
            AddressType::Holding => unwrap!(MODBUS_CONTEXT
                .lock()
                .await
                .set_holdings_from_u32(self.address_start, value)
                .map_err(|e| e as u8)),
            _ => defmt::unreachable!(),
        };

        trigger_registrations(self.address_start..=self.address_start + 1, A::TYPE).await;
    }

    pub async fn get_listener(&self) -> impl Stream<Item = u32> + '_ {
        ModbusListener::new(self.address_start..=self.address_start + 1, A::TYPE)
            .await
            .associate_u32(self)
    }
}

impl<A: RegisterAddress> ModbusRegister<u16, A> {
    pub async fn read(&self) -> u16 {
        match A::TYPE {
            AddressType::Input => unwrap!(MODBUS_CONTEXT
                .lock()
                .await
                .get_input(self.address_start)
                .map_err(|e| e as u8)),
            AddressType::Holding => unwrap!(MODBUS_CONTEXT
                .lock()
                .await
                .get_holding(self.address_start)
                .map_err(|e| e as u8)),
            _ => defmt::unreachable!(),
        }
    }

    pub async fn write(&self, value: u16) {
        match A::TYPE {
            AddressType::Input => unwrap!(MODBUS_CONTEXT
                .lock()
                .await
                .set_input(self.address_start, value)
                .map_err(|e| e as u8)),
            AddressType::Holding => unwrap!(MODBUS_CONTEXT
                .lock()
                .await
                .set_holding(self.address_start, value)
                .map_err(|e| e as u8)),
            _ => defmt::unreachable!(),
        };

        trigger_registrations(self.address_start..=self.address_start, A::TYPE).await;
    }

    pub async fn get_listener(&self) -> impl Stream<Item = u16> + '_ {
        ModbusListener::new(self.address_start..=self.address_start, A::TYPE)
            .await
            .associate_u16(self)
    }
}

pub struct Coils;
pub struct Discretes;
pub struct Inputs;
pub struct Holdings;

pub trait Address: 'static {
    const TYPE: AddressType;
}
pub trait RegisterAddress: Address {}
pub trait BitAddress: Address {}

impl Address for Coils {
    const TYPE: AddressType = AddressType::Coil;
}
impl BitAddress for Coils {}

impl Address for Discretes {
    const TYPE: AddressType = AddressType::Discrete;
}
impl BitAddress for Discretes {}

impl Address for Inputs {
    const TYPE: AddressType = AddressType::Input;
}
impl RegisterAddress for Inputs {}

impl Address for Holdings {
    const TYPE: AddressType = AddressType::Holding;
}
impl RegisterAddress for Holdings {}
