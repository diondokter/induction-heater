use core::cell::RefCell;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::ops::RangeInclusive;
use core::task::{Context, Poll, Waker};

use super::USART1;
use critical_section::Mutex;
use defmt::unwrap;
use embassy_stm32::peripherals::{DMA1_CH1, DMA1_CH2};
use embassy_stm32::usart::Uart;
use futures::{Stream, StreamExt};
use rmodbus::server::context::ModbusContext;
use rmodbus::server::ModbusFrame;
use rmodbus::ModbusFrameBuf;
use rmodbus::{consts::*, ErrorKind};

// ----- Control (holdings) -----
/// The dutycycle of the PWM that drives the coil. 0..=1
pub const COIL_POWER_DUTYCYCLE: ModbusRegister<f32, Holdings> = ModbusRegister::new(0);
// ----- Control (coils) -----
pub const COIL_POWER_ENABLE: ModbusRegister<bool, Coils> = ModbusRegister::new(0);

// ----- Report (inputs) -----
pub const COIL_DRIVE_FREQUENCY: ModbusRegister<u32, Inputs> = ModbusRegister::new(0);
pub const COIL_VOLTAGE_MAX: ModbusRegister<f32, Inputs> = ModbusRegister::new(2);
pub const FAN_RPM: ModbusRegister<u16, Inputs> = ModbusRegister::new(4);
pub const ADC_SAMPLES: ModbusRegister<[u16; crate::coil_measure::NUM_SAMPLES], Inputs> =
    ModbusRegister::new(5);
// ----- Report (discretes) -----
pub const LED_GREEN: ModbusRegister<bool, Discretes> = ModbusRegister::new(0);
pub const LED_RED: ModbusRegister<bool, Discretes> = ModbusRegister::new(1);

static MODBUS_CONTEXT: Mutex<
    RefCell<ModbusContext<1, 2, { 5 + crate::coil_measure::NUM_SAMPLES }, 2>>,
> = Mutex::new(RefCell::new(ModbusContext::new()));

static MODBUS_LISTENER_REGISTRATIONS: Mutex<RefCell<[Option<ModbusListenerRegistration>; 8]>> =
    Mutex::new(RefCell::new([REGINIT; 8]));
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
        let _rx_len = match rs485.read_until_idle(&mut receive_buffer).await {
            Ok(len) => len,
            Err(e) => {
                defmt::error!("Could not read rs485 bus: {}", e);
                continue;
            }
        };

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
            let result = critical_section::with(|cs| match frame.readonly {
                true => frame.process_read(&*MODBUS_CONTEXT.borrow_ref(cs)),
                false => frame.process_write(&mut *MODBUS_CONTEXT.borrow_ref_mut(cs)),
            });
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
                trigger_registrations(address_range, address_type);
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

fn trigger_registrations(address_range: RangeInclusive<u16>, address_type: AddressType) {
    critical_section::with(|cs| {
        MODBUS_LISTENER_REGISTRATIONS
            .borrow_ref_mut(cs)
            .iter_mut()
            .flatten()
            .for_each(|registration| {
                registration.on_context_written(address_range.clone(), address_type)
            });
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

        critical_section::with(|cs| {
            let mut registrations = MODBUS_LISTENER_REGISTRATIONS.borrow_ref_mut(cs);

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
        })
    }

    fn associate_bool<A: BitAddress + 'static>(
        self,
        register: &ModbusRegister<bool, A>,
    ) -> impl Stream<Item = bool> + '_ {
        self.then(|_| async { register.read() })
    }

    fn associate<V: RegisterValue, A: RegisterAddress + 'static>(
        self,
        register: &ModbusRegister<V, A>,
    ) -> impl Stream<Item = V> + '_ {
        self.then(|_| async { register.read() })
    }
}

impl futures::Stream for ModbusListener {
    type Item = ();

    fn poll_next(
        self: core::pin::Pin<&mut Self>,
        cx: &mut Context<'_>,
    ) -> Poll<Option<Self::Item>> {
        critical_section::with(|cs| {
            // Should work because this is a private threadmode mutex
            let mut registrations = MODBUS_LISTENER_REGISTRATIONS.borrow_ref_mut(cs);

            let waker = &mut unwrap!(registrations[self.index].as_mut()).waker;

            if waker.is_none() {
                *waker = Some(cx.waker().clone());
                Poll::Ready(Some(()))
            } else {
                Poll::Pending
            }
        })
    }
}

impl Drop for ModbusListener {
    fn drop(&mut self) {
        critical_section::with(|cs| {
            // Should work because this is a private threadmode mutex
            MODBUS_LISTENER_REGISTRATIONS.borrow_ref_mut(cs)[self.index] = None;
        });
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
    pub fn read(&self) -> bool {
        critical_section::with(|cs| {
            let ctx = MODBUS_CONTEXT.borrow_ref_mut(cs);
            match A::TYPE {
                AddressType::Coil => unwrap!(ctx.get_coil(self.address_start).map_err(|e| e as u8)),
                AddressType::Discrete => {
                    unwrap!(ctx.get_discrete(self.address_start).map_err(|e| e as u8))
                }
                _ => defmt::unreachable!(),
            }
        })
    }

    pub fn write(&self, value: bool) {
        critical_section::with(|cs| {
            let mut ctx = MODBUS_CONTEXT.borrow_ref_mut(cs);
            match A::TYPE {
                AddressType::Coil => {
                    unwrap!(ctx.set_coil(self.address_start, value).map_err(|e| e as u8))
                }
                AddressType::Discrete => unwrap!(ctx
                    .set_discrete(self.address_start, value)
                    .map_err(|e| e as u8)),
                _ => defmt::unreachable!(),
            }
        });

        trigger_registrations(self.address_start..=self.address_start, A::TYPE);
    }

    pub async fn get_listener(&self) -> impl Stream<Item = bool> + '_ {
        ModbusListener::new(self.address_start..=self.address_start, A::TYPE)
            .await
            .associate_bool::<A>(self)
    }
}

impl<A: RegisterAddress, V: RegisterValue> ModbusRegister<V, A> {
    pub fn read(&self) -> V {
        critical_section::with(|cs| match A::TYPE {
            AddressType::Input => unwrap!((V::read_input::<_, _, _, _>())(
                &*MODBUS_CONTEXT.borrow_ref(cs),
                self.address_start
            )
            .map_err(|e| e as u8)),
            AddressType::Holding => unwrap!((V::read_holding::<_, _, _, _>())(
                &*MODBUS_CONTEXT.borrow_ref(cs),
                self.address_start
            )
            .map_err(|e| e as u8)),
            _ => defmt::unreachable!(),
        })
    }

    #[inline(never)]
    pub fn write(&self, value: V) {
        critical_section::with(|cs| match A::TYPE {
            AddressType::Input => unwrap!((V::write_input::<_, _, _, _>())(
                &mut *MODBUS_CONTEXT.borrow_ref_mut(cs),
                self.address_start,
                value,
            )
            .map_err(|e| e as u8)),
            AddressType::Holding => unwrap!((V::write_holding::<_, _, _, _>())(
                &mut *MODBUS_CONTEXT.borrow_ref_mut(cs),
                self.address_start,
                value,
            )
            .map_err(|e| e as u8)),
            _ => defmt::unreachable!(),
        });

        trigger_registrations(self.address_start..=self.address_start, A::TYPE);
    }

    pub async fn get_listener(&self) -> impl Stream<Item = V> + '_ {
        ModbusListener::new(self.address_start..=self.address_start, A::TYPE)
            .await
            .associate(self)
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

pub trait RegisterValue
where
    Self: Sized,
{
    fn write_input<const C: usize, const D: usize, const I: usize, const H: usize>(
    ) -> fn(&mut ModbusContext<C, D, I, H>, u16, Self) -> Result<(), ErrorKind>;
    fn write_holding<const C: usize, const D: usize, const I: usize, const H: usize>(
    ) -> fn(&mut ModbusContext<C, D, I, H>, u16, Self) -> Result<(), ErrorKind>;
    fn read_input<const C: usize, const D: usize, const I: usize, const H: usize>(
    ) -> fn(&ModbusContext<C, D, I, H>, u16) -> Result<Self, ErrorKind>;
    fn read_holding<const C: usize, const D: usize, const I: usize, const H: usize>(
    ) -> fn(&ModbusContext<C, D, I, H>, u16) -> Result<Self, ErrorKind>;
}

impl RegisterValue for u16 {
    fn write_input<const C: usize, const D: usize, const I: usize, const H: usize>(
    ) -> fn(&mut ModbusContext<C, D, I, H>, u16, Self) -> Result<(), ErrorKind> {
        ModbusContext::set_input
    }

    fn write_holding<const C: usize, const D: usize, const I: usize, const H: usize>(
    ) -> fn(&mut ModbusContext<C, D, I, H>, u16, Self) -> Result<(), ErrorKind> {
        ModbusContext::set_holding
    }

    fn read_input<const C: usize, const D: usize, const I: usize, const H: usize>(
    ) -> fn(&ModbusContext<C, D, I, H>, u16) -> Result<Self, ErrorKind> {
        ModbusContext::get_input
    }

    fn read_holding<const C: usize, const D: usize, const I: usize, const H: usize>(
    ) -> fn(&ModbusContext<C, D, I, H>, u16) -> Result<Self, ErrorKind> {
        ModbusContext::get_holding
    }
}

impl RegisterValue for u32 {
    fn write_input<const C: usize, const D: usize, const I: usize, const H: usize>(
    ) -> fn(&mut ModbusContext<C, D, I, H>, u16, Self) -> Result<(), ErrorKind> {
        ModbusContext::set_inputs_from_u32
    }

    fn write_holding<const C: usize, const D: usize, const I: usize, const H: usize>(
    ) -> fn(&mut ModbusContext<C, D, I, H>, u16, Self) -> Result<(), ErrorKind> {
        ModbusContext::set_holdings_from_u32
    }

    fn read_input<const C: usize, const D: usize, const I: usize, const H: usize>(
    ) -> fn(&ModbusContext<C, D, I, H>, u16) -> Result<Self, ErrorKind> {
        ModbusContext::get_inputs_as_u32
    }

    fn read_holding<const C: usize, const D: usize, const I: usize, const H: usize>(
    ) -> fn(&ModbusContext<C, D, I, H>, u16) -> Result<Self, ErrorKind> {
        ModbusContext::get_holdings_as_u32
    }
}

impl RegisterValue for f32 {
    fn write_input<const C: usize, const D: usize, const I: usize, const H: usize>(
    ) -> fn(&mut ModbusContext<C, D, I, H>, u16, Self) -> Result<(), ErrorKind> {
        ModbusContext::set_inputs_from_f32
    }

    fn write_holding<const C: usize, const D: usize, const I: usize, const H: usize>(
    ) -> fn(&mut ModbusContext<C, D, I, H>, u16, Self) -> Result<(), ErrorKind> {
        ModbusContext::set_holdings_from_f32
    }

    fn read_input<const C: usize, const D: usize, const I: usize, const H: usize>(
    ) -> fn(&ModbusContext<C, D, I, H>, u16) -> Result<Self, ErrorKind> {
        ModbusContext::get_inputs_as_f32
    }

    fn read_holding<const C: usize, const D: usize, const I: usize, const H: usize>(
    ) -> fn(&ModbusContext<C, D, I, H>, u16) -> Result<Self, ErrorKind> {
        ModbusContext::get_holdings_as_f32
    }
}

impl<const N: usize> RegisterValue for [u16; N] {
    fn write_input<const C: usize, const D: usize, const I: usize, const H: usize>(
    ) -> fn(&mut ModbusContext<C, D, I, H>, u16, Self) -> Result<(), ErrorKind> {
        |ctx, address_start, value| ctx.set_inputs_bulk(address_start, &value)
    }

    fn write_holding<const C: usize, const D: usize, const I: usize, const H: usize>(
    ) -> fn(&mut ModbusContext<C, D, I, H>, u16, Self) -> Result<(), ErrorKind> {
        |ctx, address_start, value| ctx.set_holdings_bulk(address_start, &value)
    }

    fn read_input<const C: usize, const D: usize, const I: usize, const H: usize>(
    ) -> fn(&ModbusContext<C, D, I, H>, u16) -> Result<Self, ErrorKind> {
        |ctx, address_start| {
            let mut value = heapless::Vec::<_, N>::new();
            ctx.get_inputs_bulk(address_start, N as u16, &mut value)?;
            value.into_array().map_err(|_| ErrorKind::IllegalDataValue)
        }
    }

    fn read_holding<const C: usize, const D: usize, const I: usize, const H: usize>(
    ) -> fn(&ModbusContext<C, D, I, H>, u16) -> Result<Self, ErrorKind> {
        |ctx, address_start| {
            let mut value = heapless::Vec::<_, N>::new();
            ctx.get_holdings_bulk(address_start, N as u16, &mut value)?;
            value.into_array().map_err(|_| ErrorKind::IllegalDataValue)
        }
    }
}
