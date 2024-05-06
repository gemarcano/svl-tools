use byteorder::BigEndian;
use byteorder::ReadBytesExt;
use byteorder::WriteBytesExt;

use crc::{Crc, CRC_16_UMTS};
use log::debug;

use std::io;

use std::result;

pub(crate) type Result<T> = result::Result<T, Error>;

pub(crate) enum Error {
    Serial(serialport::Error),
    Io(io::Error),
    InvalidPacket(String),
    BootloadError(String),
}

impl From<serialport::Error> for Error {
    fn from(err: serialport::Error) -> Error {
        Error::Serial(err)
    }
}

impl From<io::Error> for Error {
    fn from(err: io::Error) -> Error {
        Error::Io(err)
    }
}

#[derive(Copy, Clone)]
pub(crate) enum Command {
    Version = 1,
    Bootload,
    Next,
    Frame,
    Retry,
    Done,
    Read = 100,
    ReadResponse,
}

impl From<Command> for u8 {
    fn from(command: Command) -> Self {
        command as u8
    }
}

impl TryFrom<u8> for Command {
    type Error = Error;
    fn try_from(num: u8) -> result::Result<Self, Self::Error> {
        match num {
            1 => Ok(Command::Version),
            2 => Ok(Command::Bootload),
            3 => Ok(Command::Next),
            4 => Ok(Command::Frame),
            5 => Ok(Command::Retry),
            6 => Ok(Command::Done),
            100 => Ok(Command::Read),
            101 => Ok(Command::ReadResponse),
            _ => Err(Error::InvalidPacket("Unknown command".to_string())),
        }
    }
}

// SVL uses CRC16 UMTS

pub(crate) struct Packet {
    pub command: Command,
    pub data: Vec<u8>,
}

pub(crate) trait Svl: ReadBytesExt + WriteBytesExt {
    fn get_packet(&mut self) -> Result<Packet> {
        // Packet consists of:
        // 2 bytes as length (big endian)
        // 1 byte as command
        // [0 - N] bytes as payload
        // 2 bytes as CRC (big endian)
        //
        // Minimum packet size is 3, as length bytes don't count towards packet size.

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

    fn send_packet(&mut self, packet: &Packet) -> Result<()> {
        if packet.data.len() > (u16::MAX - 3).into() {
            return Err(Error::InvalidPacket("Too much data in packet".to_string()));
        }

        let len: u16 = (packet.data.len() + 3) as u16;
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
