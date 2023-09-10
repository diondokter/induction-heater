use std::{error::Error, sync::{Arc, Mutex}, thread};

use clap::Parser;
use ih_modbus::InductionHeaterState;

mod ih_modbus;
mod tui;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
pub struct Args {
    /// Name of the serial port to use
    #[arg(short, long)]
    serial_port: String,

    /// Baud rate of the serial port
    #[arg(short, long, default_value_t = 921600)]
    baud: u32,

    /// The id of the induction heater device
    #[arg(short, long, default_value_t = 1)]
    unit_id: u8,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Arc::new(Args::parse());

    let state = Arc::new(Mutex::new(InductionHeaterState::default()));
    let state_tui = state.clone();

    thread::spawn(|| ih_modbus::run(args, state));
    tui::run_tui(state_tui).unwrap();

    Ok(())
}
