use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use getset::{CopyGetters, Setters};
use rmodbus::{client::ModbusRequest, guess_response_frame_len, ModbusProto};
use serialport::SerialPort;

use crate::Args;

#[derive(Debug, Clone, Default, CopyGetters, Setters)]
pub struct InductionHeaterState {
    #[getset(get_copy = "pub")]
    led_green: bool,
    #[getset(get_copy = "pub")]
    led_red: bool,

    #[getset(get_copy = "pub")]
    coil_drive_frequency: u32,
    #[getset(get_copy = "pub")]
    coil_voltage_max: f32,
    #[getset(get_copy = "pub")]
    fan_rpm: u16,

    #[getset(get_copy = "pub")]
    enabled: bool,
    #[getset(get_copy = "pub", set = "pub")]
    target_enabled: bool,

    // pub const COIL_DRIVE_FREQUENCY: ModbusRegister<u32, Inputs> = ModbusRegister::new(0);
    // pub const COIL_VOLTAGE_MAX: ModbusRegister<f32, Inputs> = ModbusRegister::new(2);
    // pub const FAN_RPM: ModbusRegister<u16, Inputs> = ModbusRegister::new(4);

}

pub fn run(args: Arc<Args>, state: Arc<Mutex<InductionHeaterState>>) -> Result<(), anyhow::Error> {
    let mut port = serialport::new(&args.serial_port, args.baud)
        .timeout(Duration::from_secs(1))
        .open()?;

    let mut first_loop = true;

    loop {
        // Read the data
        let discretes = read_discretes(&mut port, args.unit_id, 0, 2)?;
        let led_g = discretes[0];
        let led_r = discretes[1];

        let enabled = read_coils(&mut port, args.unit_id, 0, 1)?[0];

        let inputs = read_inputs(&mut port, args.unit_id, 0, 5)?;
        let coil_drive_frequency = (inputs[0] as u32) << 16 | inputs[1] as u32;
        let coil_voltage_max = f32::from_bits((inputs[2] as u32) << 16 | inputs[3] as u32);
        let fan_rpm = inputs[4];

        // Apply the data
        let mut state_guard = state.lock().unwrap();

        state_guard.led_green = led_g;
        state_guard.led_red = led_r;
        state_guard.enabled = enabled;
        state_guard.coil_drive_frequency = coil_drive_frequency;
        state_guard.coil_voltage_max = coil_voltage_max;
        state_guard.fan_rpm = fan_rpm;

        if first_loop {
            state_guard.target_enabled = enabled;
        }

        // Make the current state follow the targets
        let current_state = state_guard.clone();
        drop(state_guard);

        if current_state.target_enabled != enabled {
            write_coils(&mut port, args.unit_id, 0, &[current_state.target_enabled])?;
        }

        first_loop = false;
        std::thread::sleep(Duration::from_millis(20));
    }
}

fn read_discretes(
    port: &mut Box<dyn SerialPort>,
    unit_id: u8,
    reg: u16,
    count: u16,
) -> Result<Vec<bool>, anyhow::Error> {
    let mut request = ModbusRequest::new(unit_id, ModbusProto::Rtu);
    let mut buf = Vec::new();

    request.generate_get_discretes(reg, count, &mut buf)?;
    port.write_all(&buf)?;

    let mut buf = [0u8; 6];
    port.read_exact(&mut buf)?;
    let mut response = Vec::new();
    response.extend_from_slice(&buf);

    let len = guess_response_frame_len(&buf, ModbusProto::Rtu)?;
    if len > 6 {
        let mut rest = vec![0u8; (len - 6) as usize];
        port.read_exact(&mut rest)?;
        response.extend(rest);
    }

    let mut data = Vec::new();
    request.parse_bool(&response, &mut data)?;

    Ok(data)
}

fn read_inputs(
    port: &mut Box<dyn SerialPort>,
    unit_id: u8,
    reg: u16,
    count: u16,
) -> Result<Vec<u16>, anyhow::Error> {
    let mut request = ModbusRequest::new(unit_id, ModbusProto::Rtu);
    let mut buf = Vec::new();

    request.generate_get_inputs(reg, count, &mut buf)?;
    port.write_all(&buf)?;

    let mut buf = [0u8; 6];
    port.read_exact(&mut buf)?;
    let mut response = Vec::new();
    response.extend_from_slice(&buf);

    let len = guess_response_frame_len(&buf, ModbusProto::Rtu)?;
    if len > 6 {
        let mut rest = vec![0u8; (len - 6) as usize];
        port.read_exact(&mut rest)?;
        response.extend(rest);
    }

    let mut data = Vec::new();
    request.parse_u16(&response, &mut data)?;

    Ok(data)
}

fn read_coils(
    port: &mut Box<dyn SerialPort>,
    unit_id: u8,
    reg: u16,
    count: u16,
) -> Result<Vec<bool>, anyhow::Error> {
    let mut request = ModbusRequest::new(unit_id, ModbusProto::Rtu);
    let mut buf = Vec::new();

    request.generate_get_discretes(reg, count, &mut buf)?;
    port.write_all(&buf)?;

    let mut buf = [0u8; 6];
    port.read_exact(&mut buf)?;
    let mut response = Vec::new();
    response.extend_from_slice(&buf);

    let len = guess_response_frame_len(&buf, ModbusProto::Rtu)?;
    if len > 6 {
        let mut rest = vec![0u8; (len - 6) as usize];
        port.read_exact(&mut rest)?;
        response.extend(rest);
    }

    let mut data = Vec::new();
    request.parse_bool(&response, &mut data)?;

    Ok(data)
}

fn write_coils(
    port: &mut Box<dyn SerialPort>,
    unit_id: u8,
    reg: u16,
    values: &[bool],
) -> Result<(), anyhow::Error> {
    let mut request = ModbusRequest::new(unit_id, ModbusProto::Rtu);

    let mut buf = Vec::new();
    request.generate_set_coils_bulk(reg, values, &mut buf)?;
    port.write_all(&buf)?;

    // read first 6 bytes of response frame
    let mut buf = [0u8; 6];
    port.read_exact(&mut buf)?;

    let mut response = Vec::new();
    response.extend_from_slice(&buf);

    let len = guess_response_frame_len(&buf, ModbusProto::Rtu)?;
    // read rest of response frame
    if len > 6 {
        let mut rest = vec![0u8; (len - 6) as usize];
        port.read_exact(&mut rest)?;
        response.extend(rest);
    }

    // check if frame has no Modbus error inside
    request.parse_ok(&response)?;

    Ok(())
}
