// SPDX-FileCopyrightText: 2020 Ambiq Micro
// SPDX-FileCopyrightText: 2023-2024 Gabriel Marcano
//
// SPDX-License-Identifier: BSD-3-Clause

//! SVL bootloader access routines.

use byteorder::BigEndian;
use byteorder::ReadBytesExt;
use byteorder::WriteBytesExt;

use crc::{CRC_16_UMTS, Crc};
use log::debug;

use std::io;

use std::result;

pub type Result<T> = result::Result<T, Error>;

/// Errors that can be emitted by the bootloader and related functions.
pub enum Error {
    Serial(serialport::Error),
    Io(io::Error),
    InvalidPacket(String),
    Bootload(String),
}

impl From<serialport::Error> for Error {
    fn from(err: serialport::Error) -> Self {
        Self::Serial(err)
    }
}

impl From<io::Error> for Error {
    fn from(err: io::Error) -> Self {
        Self::Io(err)
    }
}

/// Commands supported by the (augmented) SVL bootloader
/// Commands with a value less than 100 are supported by the original SVL bootloader. Those
/// commands with a value of 100 or above are supported by our augmented SVL bootloader.
#[derive(Copy, Clone)]
pub enum Command {
    /// When being sent, this command requests the version of the SVL bootloader. It has no
    /// payload.
    ///
    /// When being received, this command contains the version of the SVL bootloader as a single
    /// byte in the payload.
    Version = 1,
    /// When being sent, this command requests the SVL bootloader to enter bootloader mode, and to
    /// be ready to accept other bootloader commands. It has no payload.
    ///
    /// The bootloader does not send this command back.
    Bootload,
    /// We do not send this command to the bootloader.
    ///
    /// When being received, this indicates that the bootloader is ready to receive another
    /// command. It has no payload.
    Next,
    /// When being sent, this command contains data to be flashed to the next frame on the device.
    /// This contains up to the frame size of data in the payload to be flashed (2048 bytes on the
    /// original SVL bootloader, 8192 bytes or the flash page size on the augmented SVL
    /// bootloader).
    ///
    /// The bootloader does not send this command back.
    Frame,
    /// We do not send this command to the bootloader.
    ///
    /// When being received, this indicates that something happened and the bootloader was unable
    /// to execute the previous command, and it is asking for it to be repeated. There is no
    /// payload.
    Retry,
    /// When being sent, this command informs the bootloader to jump to the application code. It
    /// has no payload.
    ///
    /// The bootloader does not send this command back.
    Done,
    /// When being sent, this command requests a number of bytes to be read from a given address.
    /// The payload consists of two 32-bit unsigned integers in network byte order. The first
    /// unsigned integer is the address to read from, and the second is the number of bytes to read
    /// from that address.
    ///
    /// The bootloader does not send this command back.
    Read = 100,
    /// We do not send this command to the bootloader.
    ///
    /// When being received, this contains the data requested in the previous Read command. The
    /// payload should contain the number of bytes requested in the previous Read command.
    ReadResponse,
}

impl From<Command> for u8 {
    fn from(command: Command) -> Self {
        command as Self
    }
}

impl TryFrom<u8> for Command {
    type Error = Error;
    fn try_from(num: u8) -> result::Result<Self, Self::Error> {
        match num {
            1 => Ok(Self::Version),
            2 => Ok(Self::Bootload),
            3 => Ok(Self::Next),
            4 => Ok(Self::Frame),
            5 => Ok(Self::Retry),
            6 => Ok(Self::Done),
            100 => Ok(Self::Read),
            101 => Ok(Self::ReadResponse),
            _ => Err(Error::InvalidPacket("Unknown command".to_string())),
        }
    }
}

/// Represents a packet of bootloader data.
pub struct Packet {
    /// The command associated with the packet.
    pub command: Command,
    /// An optional payload, its meaning depends on the command.
    pub data: Vec<u8>,
}

/// A trait for sending to and receiving data from the SVL bootloader.
///
/// A raw packet consists of the following:
///  - 2 bytes as length (big endian)
///  - 1 byte as command
///  - [0 - N] bytes as payload
///  - 2 bytes as CRC (big endian)
///
/// The length specifies how many bytes there are in the rest of the packet, including CRC.
///
/// The CRC algorithm used is CRC16 UMTS.
///
/// The minimum packet size is 3, as length bytes don't count towards packet size.
///
/// All data is encoded in network byte order.
pub trait Svl: ReadBytesExt + WriteBytesExt {
    /// Gets a packet from the bootloader.
    ///
    /// # Errors
    ///
    /// [Error::InvalidPacket] If the CRC received doesn't match the data, or if the packet length
    /// is less than three.
    fn get_packet(&mut self) -> Result<Packet> {
        let len = self.read_u16::<BigEndian>()?;
        debug!("Received a packet len: {}", len);
        if len < 3 {
            return Err(Error::InvalidPacket(
                "Packet length is too short".to_string(),
            ));
        }

        // Payload consists of command, data and CRC
        let mut payload = vec![0u8; len.into()];
        self.read_exact(&mut payload)?;

        // If CRC is correct, calculating the CRC over the entire payload, CRC16 inclusive, should
        // yield 0.
        let crc = Crc::<u16>::new(&CRC_16_UMTS);
        let mut digest = crc.digest();
        digest.update(&payload);
        if digest.finalize() != 0 {
            return Err(Error::InvalidPacket("CRC failed".to_string()));
        }

        debug!("Command received: {}", payload[0]);
        debug!("Data length: {}", (len - 2) - 1);
        let result = Packet {
            command: payload[0].try_into()?,
            data: payload[1..(len - 2).into()].to_vec(),
        };
        Ok(result)
    }

    /// Sends a packet to the bootloader.
    ///
    /// # Errors
    /// [Error::InvalidPacket] if the payload length is larger than what the length field supports.
    fn send_packet(&mut self, packet: &Packet) -> Result<()> {
        if packet.data.len() > (u16::MAX - 3).into() {
            return Err(Error::InvalidPacket("Too much data in packet".to_string()));
        }

        let len: u16 = u16::try_from(packet.data.len() + 3).unwrap();
        let cmd: u8 = packet.command.into();
        let crc = Crc::<u16>::new(&CRC_16_UMTS);
        let mut digest = crc.digest();
        digest.update(&[cmd]);
        digest.update(&packet.data);
        let crc = digest.finalize();

        debug!("Sending len: {}, cmd: {}, crc: 0x{:04X}", len, cmd, crc);
        self.write_u16::<BigEndian>(len)?;
        self.write_u8(cmd)?;
        self.write_all(&packet.data)?;
        self.write_u16::<BigEndian>(crc)?;

        Ok(())
    }
}

impl<T: ReadBytesExt + WriteBytesExt + ?Sized> Svl for T {}
