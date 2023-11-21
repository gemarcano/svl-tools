mod svl;
use svl::Command;
use svl::Error;
use svl::Packet;
use svl::Result;
use svl::Svl;

use clap::Parser;
use log::{debug, error, info};
use serialport::SerialPort;
use indicatif::{ProgressBar, ProgressStyle};

use std::cmp;
use std::fs::File;
use std::io::Read;
use std::num;
use std::path::Path;
use std::path::PathBuf;
use std::result;
use std::time;
use std::time::Duration;
use std::time::Instant;
use std::fs;

fn phase_setup(serial: &mut (impl SerialPort + Svl + ?Sized)) -> Result<()> {
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

    println!("Got SVL Bootloader Version: {}", packet.data[0]);
    info!("Sending 'enter bootloader' command");

    serial.send_packet(&Packet {
        command: Command::Bootload,
        data: vec![],
    })?;

    Ok(())
}

fn phase_bootload(serial: &mut (impl Svl + ?Sized), bin_path: &Path) -> Result<()> {
    info!("Phase: Bootload");

    let start_time = Instant::now();
    // The frame size is determined by the size of the receive buffer on the device, which is 2048
    // currently
    let frame_size: usize = 512 * 4;

    let resend_max = 4;
    let mut resend_count = 0;

    let mut binary = File::open(bin_path)?;
    let mut binary_data = vec![];

    //FIXME should we first read the size of the data, and make a determination about whether it is
    //OK to send?
    let total_length = fs::metadata(bin_path)?.len() as usize;
    // FIXME why not iterate on the data?
    binary.read_to_end(&mut binary_data)?;

    assert!(total_length == binary_data.len());
    let total_frames = ((total_length as f64) / (frame_size as f64)).ceil() + 0.5;
    let total_frames = total_frames as usize;

    let mut current_frame: usize = 0;

    let progress_bar = ProgressBar::new(total_frames as u64);
    progress_bar
        .set_style(ProgressStyle::with_template("{bar:^20.red/white.bold} {percent:>3}%").unwrap());
    progress_bar.tick();

    info!(
        "have {} bytes to send in {} frames",
        total_length, total_frames
    );

    let mut bootloader_done = false;

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
                    return Err(Error::BootloadError("Too many retries...".to_string()));
                }
            }
            _ => {
                return Err(Error::BootloadError("Timeout or unknown error".to_string()));
            }
        }

        debug!(
            "Current frame: {}, Resend count: {}",
            current_frame, resend_count
        );
        if current_frame <= total_frames {
            // If we knew what our maximum physical limits were, we can guarantee conversions...
            // Is this bootloader only used on the Apollo3 Sparkfun boards???
            // If so, we can check that the binary fits in flash, so we can confirm that all of the
            // conversions here are valid, at least for a 32-bit host platform...
            // There's no way a 16 bit plaform can support loading in a large binary all at once,
            // so just unwrap frame start and end
            let frame_start = (current_frame - 1) * frame_size;
            let frame_end = frame_start + cmp::min(frame_size, total_length - frame_start);
            debug!("frame start: {}, frame_end: {}", frame_start, frame_end);
            let frame_data = &binary_data[frame_start..frame_end];

            info!(
                "Sending frame# {}, length: {}",
                current_frame,
                frame_data.len()
            );

            progress_bar.set_position(current_frame as u64);

            serial.send_packet(&Packet {
                command: Command::Frame,
                data: frame_data.to_vec(),
            })?;
        } else {
            serial.send_packet(&Packet {
                command: Command::Done,
                data: vec![],
            })?;
            bootloader_done = true;
        }
    }

    println!("Upload complete");
    let bits_per_second = (total_length as f64) / (Instant::now() - start_time).as_secs_f64();
    println!("Nominal bootload bps: {}", bits_per_second);

    Ok(())
}

#[derive(Parser)]
#[command(author, version, about, long_about=None)]
struct Cli {
    port: PathBuf,
    #[arg(short, long, default_value_t = 115200)]
    baud: u32,
    #[arg(short = 'f', long)]
    binfile: PathBuf,
    #[command(flatten)]
    verbose: clap_verbosity_flag::Verbosity,
    #[arg(short, long, value_parser=parse_duration, default_value="500")]
    timeout: Duration,
}

fn parse_duration(arg: &str) -> result::Result<Duration, num::ParseIntError> {
    let milliseconds = arg.parse()?;
    Ok(std::time::Duration::from_millis(milliseconds))
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

    if !cli.binfile.exists() {
        error!("Bin file {} does not exist.", cli.binfile.display());
        return;
    }
    // FIXME what about tty?

    let num_tries = 3;

    let mut entered_bootloader = false;

    for _ in 0..num_tries {
        let serial = serialport::new(cli.port.to_string_lossy(), cli.baud)
            .timeout(cli.timeout)
            .open();
        if serial.is_err() {
            error!("Unable to open serial port {}.", cli.port.display());
            return;
        }
        let mut serial = serial.unwrap();

        // Startup time for Artemis bootloader (experimentally determined - 0.095 sec min delay)
        std::thread::sleep(time::Duration::from_millis(150));

        // Pre-check-- make sure the binfile exists, and that its size is reasonable

        if phase_setup(&mut *serial).is_ok() {
            entered_bootloader = true;
            let result = phase_bootload(&mut *serial, &cli.binfile);
            match result {
                Ok(()) => break,
                Err(error) => {
                    match error {
                        Error::InvalidPacket(error) => println!("{}", error),
                        Error::BootloadError(error) => println!("{}", error),
                        Error::Io(error) => println!("{}", error),
                        Error::Serial(error) => println!("{}", error),
                    }
                    println!("Upload failed");
                }
            }
        } else {
            error!("Failed to enter bootload phase");
        }
    }

    if !entered_bootloader {
        println!("Target failed to enter bootload mode. Verify the right COM port is selected and that your board has the SVL bootloader.");
    }
}
