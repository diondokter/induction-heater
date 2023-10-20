use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use getset::{CopyGetters, Getters, Setters};
use rmodbus::{client::ModbusRequest, guess_response_frame_len, ModbusProto};
use serialport::SerialPort;

use crate::{tui::GraphSelection, Args};

#[derive(Debug, Clone, CopyGetters, Setters, Getters)]
pub struct InductionHeaterState {
    #[getset(get = "pub")]
    error: Option<Arc<anyhow::Error>>,

    #[getset(get_copy = "pub")]
    led_green: bool,
    #[getset(get_copy = "pub")]
    led_red: bool,

    #[getset(get_copy = "pub")]
    coil_drive_frequency: u32,
    coil_drive_frequencies: Vec<(f64, f64)>,

    #[getset(get_copy = "pub")]
    coil_voltage_max: f32,
    coil_voltage_maxs: Vec<(f64, f64)>,

    #[getset(get_copy = "pub")]
    fan_rpm: u16,
    fan_rpms: Vec<(f64, f64)>,

    #[getset(get_copy = "pub")]
    enabled: bool,
    #[getset(get_copy = "pub", set = "pub")]
    target_enabled: bool,

    adc_samples: Vec<(f64, f64)>,

    #[getset(get_copy = "pub", set = "pub")]
    coil_power_dutycycle: f32,
    #[getset(get_copy = "pub", set = "pub")]
    coil_power_dutycycle_updated: bool,
}

impl Default for InductionHeaterState {
    fn default() -> Self {
        Self {
            error: Default::default(),
            led_green: Default::default(),
            led_red: Default::default(),
            coil_drive_frequency: Default::default(),
            coil_drive_frequencies: Default::default(),
            coil_voltage_max: Default::default(),
            coil_voltage_maxs: Default::default(),
            fan_rpm: Default::default(),
            fan_rpms: Default::default(),
            enabled: Default::default(),
            target_enabled: Default::default(),
            adc_samples: Default::default(),
            coil_power_dutycycle: 0.5,
            coil_power_dutycycle_updated: Default::default(),
        }
    }
}

impl InductionHeaterState {
    pub fn get_data(&self, selection: GraphSelection) -> &[(f64, f64)] {
        match selection {
            GraphSelection::CoilDriveFrequency => &self.coil_drive_frequencies,
            GraphSelection::CoilVoltageMax => &self.coil_voltage_maxs,
            GraphSelection::FanRpm => &self.fan_rpms,
            GraphSelection::AdcSamples => &self.adc_samples,
        }
    }
}

pub fn run(args: Arc<Args>, state: Arc<Mutex<InductionHeaterState>>) -> ! {
    loop {
        let error = run_inner(args.clone(), state.clone()).unwrap_err();
        state.lock().unwrap().error = Some(Arc::new(error));
        std::thread::sleep(Duration::from_millis(500));
    }
}

fn run_inner(
    args: Arc<Args>,
    state: Arc<Mutex<InductionHeaterState>>,
) -> Result<(), anyhow::Error> {
    let mut port = serialport::new(&args.serial_port, args.baud)
        .timeout(Duration::from_millis(100))
        .open()
        .map_err(|e| anyhow::anyhow!("Error opening serial port at {}: {e}", &args.serial_port))?;

    let mut loop_iteration = state
        .lock()
        .unwrap()
        .coil_drive_frequencies
        .last()
        .map(|(loop_iter, _)| *loop_iter as u64)
        .unwrap_or_default();

    loop {
        // Read the data
        let discretes = read_discretes(&mut port, args.unit_id, 0, 2)?;
        let led_g = discretes[0];
        let led_r = discretes[1];

        let enabled = read_coils(&mut port, args.unit_id, 0, 1)?[0];

        const NUM_SAMPLES: usize = 64;
        let inputs = read_inputs(&mut port, args.unit_id, 0, 5 + NUM_SAMPLES as u16)?;
        let coil_drive_frequency = (inputs[0] as u32) << 16 | inputs[1] as u32;
        let coil_voltage_max = f32::from_bits((inputs[2] as u32) << 16 | inputs[3] as u32);
        let fan_rpm = inputs[4];
        let adc_samples = inputs[5..][..NUM_SAMPLES]
            .iter()
            .enumerate()
            .map(|(i, sample)| (i as f64, *sample as f64))
            .collect();

        // Apply the data
        let mut state_guard = state.lock().unwrap();

        state_guard.error = None;
        state_guard.led_green = led_g;
        state_guard.led_red = led_r;
        state_guard.enabled = enabled;
        state_guard.coil_drive_frequency = coil_drive_frequency;
        state_guard
            .coil_drive_frequencies
            .push((loop_iteration as f64, coil_drive_frequency as f64));
        if state_guard.coil_drive_frequencies.len() > 500 {
            state_guard.coil_drive_frequencies.remove(0);
        }
        state_guard.coil_voltage_max = coil_voltage_max;
        state_guard
            .coil_voltage_maxs
            .push((loop_iteration as f64, coil_voltage_max as f64));
        if state_guard.coil_voltage_maxs.len() > 500 {
            state_guard.coil_voltage_maxs.remove(0);
        }
        state_guard.fan_rpm = fan_rpm;
        state_guard
            .fan_rpms
            .push((loop_iteration as f64, fan_rpm as f64));
        if state_guard.fan_rpms.len() > 500 {
            state_guard.fan_rpms.remove(0);
        }

        state_guard.adc_samples = adc_samples;

        if loop_iteration == 0 {
            state_guard.target_enabled = enabled;
        }

        // Make the current state follow the targets
        let current_state = state_guard.clone();
        state_guard.coil_power_dutycycle_updated = false;
        drop(state_guard);

        if current_state.target_enabled != enabled {
            write_coils(&mut port, args.unit_id, 0, &[current_state.target_enabled])?;
        }

        if current_state.coil_power_dutycycle_updated {
            write_holdings(
                &mut port,
                args.unit_id,
                0,
                &[
                    (current_state.coil_power_dutycycle.to_bits() >> 16 & 0xFFFF) as u16,
                    (current_state.coil_power_dutycycle.to_bits() & 0xFFFF) as u16,
                ],
            )?;
        }

        loop_iteration += 1;
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

fn write_holdings(
    port: &mut Box<dyn SerialPort>,
    unit_id: u8,
    reg: u16,
    values: &[u16],
) -> Result<(), anyhow::Error> {
    let mut request = ModbusRequest::new(unit_id, ModbusProto::Rtu);

    let mut buf = Vec::new();
    request.generate_set_holdings_bulk(reg, values, &mut buf)?;
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
