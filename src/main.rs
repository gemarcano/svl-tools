// SPDX-FileCopyrightText: 2020 Ambiq Micro
// SPDX-FileCopyrightText: 2023-2024 Gabriel Marcano
//
// SPDX-License-Identifier: BSD-3-Clause

//! # svl-tools
//!
//! This CLI provides tools for interfacing with Sparkfun Variable Loader found on Sparkfun's
//! Artemis modules. This tool is derived from Sparkfun's svl.py implementation, and extended with
//! a few extra commands.
//!
//! Commands supported: flash, read

mod svl;
use svl::Command;
use svl::Error;
use svl::Packet;
use svl::Result;
use svl::Svl;

use clap::builder::{PossibleValuesParser, TypedValueParser};
use clap::{Parser, Subcommand};
use indicatif::{ProgressBar, ProgressStyle};
use log::{debug, error, info};
use serialport::SerialPort;

use std::cmp;
use std::fs::File;
use std::io::BufRead;
use std::io::BufReader;
use std::num;
use std::path::Path;
use std::path::PathBuf;
use std::result;
use std::time;
use std::time::Duration;
use std::time::Instant;

fn phase_setup(serial: &mut (impl SerialPort + Svl + ?Sized)) -> Result<u8> {
    info!("Phase: Setup");

    // Handle the serial startup blip
    // Gabe: From what I can tell, supposedly according to the SVL bootloader source code, there's
    // apparently a 23 microsecond blip in the TX line that might be interpreted as data when the
    // bootloader enables its uart interface. This just makes sure to ditch that data.
    // FIXME why is this before the baud detection? Is it because the UART TX line is inactive at
    // first during bootrom, and is either floating or... something? UART hardware isn't connected
    // until after baud detection.
    serial.clear(serialport::ClearBuffer::Input)?;
    info!("Cleared the startup blip");

    // Send the baud detection character, it's 0b10101010
    serial.write_u8(b'U')?;
    let packet = serial.get_packet()?;
    println!("Got SVL Bootloader Version: {:02X}", packet.data[0]);

    Ok(packet.data[0])
}

fn enter_bootload(serial: &mut (impl SerialPort + Svl + ?Sized)) -> Result<()> {
    info!("Sending 'enter bootloader' command");

    serial.send_packet(&Packet {
        command: Command::Bootload,
        data: vec![],
    })?;

    Ok(())
}

fn request_read(serial: &mut (impl Svl + ?Sized), address: u32, size: u16) -> Result<()> {
    info!("Sending 'read' command");

    let mut payload = vec![];
    payload.extend_from_slice(&address.to_be_bytes());
    payload.extend_from_slice(&size.to_be_bytes());
    serial.send_packet(&Packet {
        command: Command::Read,
        data: payload,
    })?;

    Ok(())
}

fn phase_read(serial: &mut (impl Svl + ?Sized), address: u32, size: u16) -> Result<()> {
    info!("Phase: Read");

    // The frame size is determined by the size of the receive buffer on the device, which is 2048
    // currently
    let resend_max = 4;
    let mut resend_count = 0;

    let mut read_done = false;

    let mut read_response = vec![];
    while !read_done {
        let packet = serial.get_packet()?;
        match packet.command {
            Command::Next | Command::Retry => {
                if matches!(packet.command, Command::Retry) {
                    if resend_count >= resend_max {
                        return Err(Error::Bootload("Too many retries...".to_string()));
                    }
                    resend_count += 1;
                    info!("Retrying...");
                }
                if request_read(&mut *serial, address, size).is_err() {
                    error!("Failed to request read");
                }
            }
            Command::ReadResponse => {
                if packet.data.len() < 4 && (packet.data.len() % 4) != 0 {
                    return Err(Error::InvalidPacket(
                        "Wrong number of bytes in payload".to_string(),
                    ));
                }
                read_response = packet.data;
                read_done = true;
            }
            _ => {
                return Err(Error::Bootload("Timeout or unknown error".to_string()));
            }
        }
    }

    print!("{size} bytes at address {address:#0X}:\n\t");
    for byte in &read_response {
        print!("{byte:#04X} ");
    }
    println!();
    // Tell bootloader we're done...
    serial.send_packet(&Packet {
        command: Command::Done,
        data: vec![],
    })?;

    Ok(())
}

fn phase_bootload(
    serial: &mut (impl Svl + ?Sized),
    bin_path: &Path,
    frame_size: u16,
) -> Result<()> {
    info!("Phase: Bootload");

    let resend_max = 4;
    let mut resend_count = 0;

    let binary = File::open(bin_path)?;

    // Max allowable size is 0xF40000 bytes, or the difference between the load address and the
    // rest of flash space. This is more than 16 bits, so we must represent the size as u32 at
    // least
    let total_length = binary.metadata()?.len();
    if total_length > (0x0010_0000 - 0xC000) {
        return Err(Error::Bootload("binary is too large".to_string()));
    }
    let total_length = u32::try_from(total_length).unwrap();

    let mut reader = BufReader::with_capacity(frame_size.into(), binary);

    // There can be no more than 0x0010_0000 / 0x100 frames, or 0x100, which is fits in a u16
    let total_frames = if total_length % u32::from(frame_size) != 0 {
        total_length / u32::from(frame_size) + 1
    } else {
        total_length / u32::from(frame_size)
    };

    let total_frames = u16::try_from(total_frames);

    if total_frames.is_err() {
        return Err(Error::Bootload(
            "total number of frames exceed limits".to_string(),
        ));
    }
    let total_frames = total_frames.unwrap();

    let mut current_frame: u16 = 0;

    let progress_bar = ProgressBar::new(total_frames.into());
    progress_bar
        .set_style(ProgressStyle::with_template("{bar:^20.red/white.bold} {percent:>3}%").unwrap());
    progress_bar.tick();

    info!(
        "have {} bytes to send in {} frames",
        total_length, total_frames
    );

    let mut bootloader_done = false;

    let start_time = Instant::now();
    while !bootloader_done {
        let packet = serial.get_packet()?;
        match packet.command {
            Command::Next => {
                current_frame += 1;
                resend_count = 0;
            }
            Command::Retry => {
                info!("Retrying...");
                resend_count += 1;
                if resend_count >= resend_max {
                    return Err(Error::Bootload("Too many retries...".to_string()));
                }
            }
            _ => {
                return Err(Error::Bootload("Timeout or unknown error".to_string()));
            }
        }

        debug!(
            "Current frame: {}, Resend count: {}",
            current_frame, resend_count
        );
        if current_frame <= total_frames {
            let frame_start = u32::from((current_frame - 1) * frame_size);
            let frame_end: u32 =
                frame_start + cmp::min(u32::from(frame_size), total_length - frame_start);
            debug!("frame start: {}, frame_end: {}", frame_start, frame_end);
            let buffer = reader.fill_buf()?;
            let buffer_length = buffer.len();

            info!(
                "Sending frame# {}, length: {}",
                current_frame, buffer_length
            );

            progress_bar.set_position(u64::from(current_frame));

            serial.send_packet(&Packet {
                command: Command::Frame,
                data: buffer.to_vec(),
            })?;
            reader.consume(buffer_length);
        } else {
            serial.send_packet(&Packet {
                command: Command::Done,
                data: vec![],
            })?;
            bootloader_done = true;
        }
    }

    let bits_per_second = f64::from(total_length) / start_time.elapsed().as_secs_f64();
    println!("Upload complete");
    println!("Nominal bootload bps: {bits_per_second}");

    Ok(())
}

fn parse_baud(arg: String) -> u32 {
    arg.parse::<u32>().unwrap()
}

const BAUD_OPTIONS: [&str; 5] = ["921600", "460800", "230400", "115200", "57600"];

#[derive(Subcommand)]
enum Commands {
    /// Flash an application through the SVL.
    Flash {
        #[arg(short = 'f', long)]
        /// The binary file to upload to the bootloader.
        binfile: PathBuf,
    },
    /// Read memory by querying the SVL.
    Read {
        #[arg(value_parser = parse_maybe_hex)]
        /// The starting address to read from.
        address: u32,
        #[arg(value_parser = parse_maybe_hex16)]
        /// The number of bytes to read.
        size: u16,
    },
}

#[derive(Parser)]
#[command(author, version, about, long_about=None)]
#[command(propagate_version = true)]
struct Cli {
    #[arg(short, long)]
    /// Path to the serial device connected to the Sparkfun Variable Loader.
    port: PathBuf,
    #[arg(
        short,
        long,
        value_parser = PossibleValuesParser::new(BAUD_OPTIONS).map(parse_baud),
        default_value_t = 921600)]
    /// The baud rate to connect to the bootloader with.
    baud: u32,
    #[command(subcommand)]
    /// The task to perform.
    command: Commands,
    #[command(flatten)]
    /// The level of output verbosity.
    verbose: clap_verbosity_flag::Verbosity,
    #[arg(short, long, value_parser=parse_duration, default_value="500")]
    /// Serial communication timeout in milliseconds.
    timeout: Duration,
}

fn parse_duration(arg: &str) -> result::Result<Duration, num::ParseIntError> {
    let milliseconds = arg.parse()?;
    Ok(Duration::from_millis(milliseconds))
}

fn parse_maybe_hex(arg: &str) -> result::Result<u32, num::ParseIntError> {
    if let Some(hex) = arg.strip_prefix("0x") {
        u32::from_str_radix(hex, 16)
    } else {
        arg.parse::<u32>()
    }
}

fn parse_maybe_hex16(arg: &str) -> result::Result<u16, num::ParseIntError> {
    if let Some(hex) = arg.strip_prefix("0x") {
        u16::from_str_radix(hex, 16)
    } else {
        arg.parse::<u16>()
    }
}

fn main() {
    let cli = Cli::parse();

    env_logger::Builder::new()
        .filter_level(cli.verbose.log_level_filter())
        .init();

    println!("Artemis SVL Bootloader");
    info!(
        "Script version: {}.{}",
        env!("CARGO_PKG_VERSION_MAJOR"),
        env!("CARGO_PKG_VERSION_MINOR")
    );

    let mut entered_bootloader = false;

    // The number of tries to try to perform the requested task...
    const NUM_TRIES: i32 = 3;
    for _ in 0..NUM_TRIES {
        let serial = serialport::new(cli.port.to_string_lossy(), cli.baud)
            .timeout(cli.timeout)
            .open();
        if serial.is_err() {
            error!("Unable to open serial port {}.", cli.port.display());
            continue;
        }
        let mut serial = serial.unwrap();

        // Startup time for Artemis bootloader (experimentally determined - 0.095 sec min delay)
        std::thread::sleep(time::Duration::from_millis(150));

        // Pre-check-- make sure the binfile exists, and that its size is reasonable
        let version = phase_setup(&mut *serial);
        let Ok(version) = version else {
            error!("Failed to setup connection");
            continue;
        };
        // Decide on the device buffer size based on the version. For Sparkfun bootloaders, use
        // 2048. For ours, use flash page size, or 8192. We have set the upper nibble for our
        // versions
        let frame_size = if (version >> 4) > 0 { 8192 } else { 2048 };

        // Send bootloader command and check for a NEXT reply...
        if enter_bootload(&mut *serial).is_ok() {
            entered_bootloader = true;

            match &cli.command {
                Commands::Flash { binfile } => {
                    if !binfile.exists() {
                        error!("Bin file {} does not exist.", binfile.display());
                        return;
                    }
                    let result = phase_bootload(&mut *serial, binfile, frame_size);
                    match result {
                        Ok(()) => break,
                        Err(error) => {
                            match error {
                                Error::InvalidPacket(error) => error!("{error}"),
                                Error::Bootload(error) => error!("{error}"),
                                Error::Io(error) => error!("{error}"),
                                Error::Serial(error) => error!("{error}"),
                            }
                            error!("Upload failed");
                            continue;
                        }
                    }
                }
                Commands::Read { address, size } => {
                    let result = phase_read(&mut *serial, *address, *size);
                    match result {
                        Ok(()) => {}
                        Err(error) => {
                            match error {
                                Error::InvalidPacket(error) => error!("{error}"),
                                Error::Bootload(error) => error!("{error}"),
                                Error::Io(error) => error!("{error}"),
                                Error::Serial(error) => error!("{error}"),
                            }
                            error!("Read failed");
                            continue;
                        }
                    }
                }
            }
            // If we got here, we didn't hit an error, so break out...
            break;
        }
        error!("Failed to enter bootload phase");
    }
    if !entered_bootloader {
        error!("Target failed to enter bootload mode. Verify the right COM port is selected and that your board has the SVL bootloader.");
    }
}
