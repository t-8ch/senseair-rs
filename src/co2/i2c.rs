//! Carbon dioxide sensors via the i2c protocol

// docs from https://rmtplusstoragesenseair.blob.core.windows.net/docs/Dev/publicerat/TDE4700.pdf

use core::convert::TryInto;

/// Default i2c address after production.
pub const DEFAULT_ADDRESS: u8 = 0x68;
/// Address responded to by device disregarding configured address.
pub const ANY_SENSOR_ADDRESS: u8 = 0x7F;

#[derive(Debug, PartialEq)]
pub enum Error {
    /// Device reported incomplete measurement
    Incomplete,
    /// Mismatch of response checksum
    ChecksumMismatch,
}

pub type Result<T> = core::result::Result<T, Error>;

/// Characteristics of a device. Can be used to implement custom device types.
pub struct Characteristics {
    meter_control_eeprom_address: Option<u16>,
}

fn checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0, |acc, x| acc.overflowing_add(*x).0)
}

enum CommandType {
    // WriteRAM = 0x1,
    ReadRAM = 0x2,
    // WriteEE = 0x3,
    ReadEE = 0x4,
}

/// Error status of a device
#[derive(Debug)]
pub enum ErrorStatus {
    FatalError = 0,
    OffsetRegulationError = 1,
    AlgorithmError = 2,
    OutputError = 3,
    SelfDiagnosticError = 4,
    OutOfRangeError = 5,
    MemoryError = 6,
    UnknownError = 7,
}

fn first_set_bit(u: u8) -> Option<u8> {
    for index in 0..8 {
        if u & (1 << index) != 0 {
            return Some(index);
        }
    }
    None
}

impl ErrorStatus {
    fn from_error(code: u8) -> Option<ErrorStatus> {
        first_set_bit(code).map(|err| match err {
            0 => Self::FatalError,
            1 => Self::OffsetRegulationError,
            2 => Self::AlgorithmError,
            3 => Self::OutputError,
            4 => Self::SelfDiagnosticError,
            5 => Self::OutOfRangeError,
            6 => Self::MemoryError,
            7 => Self::UnknownError,
            _ => unreachable!(),
        })
    }

    /// Set bit in error message returned by device
    pub fn error_bit(self) -> u8 {
        self as u8
    }
}

macro_rules! build_command {
    ( $c:expr, $b:expr, $a:expr, $cb:expr ) => {{
        let request = prepare_command($c, $b, $a);
        (request, $cb)
    }};
}

fn prepare_command(command: CommandType, bytes: u8, address: u16) -> [u8; 4] {
    let command_num = command as u8;
    assert!(command_num < 16);
    assert!(bytes <= 16);

    let address_bytes = address.to_be_bytes();
    let mut request = [
        (command_num << 4) | if bytes == 16 { 0 } else { bytes },
        address_bytes[0],
        address_bytes[1],
        0,
    ];
    request[3] = checksum(&request[0..3]);
    request
}

fn validate_checksum(data: &[u8]) -> bool {
    data[data.len() - 1] == checksum(&data[0..data.len() - 1])
}

macro_rules! parse_response {
    ( $d:expr, $b:expr  ) => {
        if !validate_checksum($d) {
            Err(Error::ChecksumMismatch)
        } else {
            let operation_status = $d[0];

            if (operation_status & 0x01) != 1 {
                Err(Error::Incomplete)
            } else {
                let slice = &$d[1..=$b];
                // FIXME does this copy?
                let array: [u8; $b] = slice.try_into().unwrap();
                Ok(array)
            }
        }
    };
}

#[derive(Debug)]
pub struct MeterControl {
    /// Automatic Background Calibration enabled
    pub abc: bool,
    /// Fractional Algorithm enabled
    pub frac_algo: bool,
    /// Dynamic Fractional Algorithm enabled
    pub dyn_frac_algo: bool,
}

fn parse_i16_response(data: &[u8; 4]) -> Result<i16> {
    let payload = parse_response!(data, 2)?;
    Ok(i16::from_be_bytes(payload))
}

fn parse_u16_response(data: &[u8; 4]) -> Result<u16> {
    let payload = parse_response!(data, 2)?;
    Ok(u16::from_be_bytes(payload))
}

fn parse_error_status(data: &[u8; 3]) -> Result<Option<ErrorStatus>> {
    let payload = parse_response!(data, 1)?[0];
    Ok(ErrorStatus::from_error(payload))
}

fn parse_meter_control(data: &[u8; 3]) -> Result<MeterControl> {
    let payload = parse_response!(data, 1)?[0];
    Ok(MeterControl {
        abc: (payload & 0b001) == 0,
        frac_algo: (payload & 0b010) == 0,
        dyn_frac_algo: (payload & 0b100) == 0,
    })
}

/**
 * Tuple of raw command payload and response parser.
 *
 * First element is the payload to be sent to the device.
 * Second element is a parser function to process the response from the device.
 */
pub type Command<T, R> = ([u8; 4], fn(&R) -> Result<T>);

/**
 * Protocol for a specific device
 *
 * The protocol does *not* provide any IO.
 * It only provides the raw request payload and parsers for the response data.
 * You have to use an i2c library (like [i2cdev](https://crates.io/crates/i2cdev)) to perform the
 * actual I/O.
 *
 * Example:
 * ```
 * # use senseair::co2::i2c::Protocol;
 * # use std::io::Write;
 * # use std::io::Read;
 * # let mut i2c: Vec<u8> = Vec::new();
 * #
 * // let i2c = some sort of i2c device
 *
 * // New protocol
 * let k30 = Protocol::k30();
 *
 * // Create command
 * let (request, cb) = k30.co2_ppm();
 *
 * assert_eq!(request, [0x22, 0x00, 0x08, 0x2A]);
 *
 * // Send command to device
 * i2c.write(&request);
 *
 * # let mut i2c = vec![0x01, 0x12, 0x34, 0x47];
 * # let mut i2c = i2c.as_slice();
 * #
 * // Read response bytes from i2c device
 * let mut response = [0; 4];
 * i2c.read(&mut response);
 *
 * // Parsing response data
 * let co2_ppm = cb(&response);
 *
 * assert_eq!(co2_ppm, Ok(4660));
 * ```
 */
pub struct Protocol {
    characteristics: Characteristics,
}

impl Protocol {
    /// Protocol for a device with given characteristics
    pub fn new(characteristics: Characteristics) -> Protocol {
        Protocol { characteristics }
    }

    /// Protocol for K30 devices
    pub fn k30() -> Protocol {
        Protocol::new(Characteristics {
            meter_control_eeprom_address: Some(0x3E),
        })
    }

    /// Measured CO2 parts per million
    pub fn co2_ppm(&self) -> Command<i16, [u8; 4]> {
        build_command!(CommandType::ReadRAM, 2, 0x08, parse_i16_response)
    }

    /// Error status of the sensor
    pub fn error_status(&self) -> Command<Option<ErrorStatus>, [u8; 3]> {
        build_command!(CommandType::ReadRAM, 1, 0x1E, parse_error_status)
    }

    /// Automatic Background calibration interval in hours
    pub fn abc_period_hours(&self) -> Command<u16, [u8; 4]> {
        build_command!(CommandType::ReadEE, 2, 0x40, parse_u16_response)
    }

    /// Zero Trim value
    pub fn zero_trim(&self) -> Command<i16, [u8; 4]> {
        build_command!(CommandType::ReadEE, 2, 0x48, parse_i16_response)
    }

    /// Meter Control information
    pub fn meter_control(&self) -> Option<Command<MeterControl, [u8; 3]>> {
        let addr = self.characteristics.meter_control_eeprom_address;
        Some(build_command!(
            CommandType::ReadEE,
            1,
            addr?,
            parse_meter_control
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_first_set_bit() {
        assert_eq!(None, first_set_bit(0b0));
        assert_eq!(Some(0), first_set_bit(0b1));
        assert_eq!(Some(0), first_set_bit(0b1001));
        assert_eq!(Some(2), first_set_bit(0b1100));
    }

    #[test]
    fn test_checksum() {
        assert_eq!(0x00, checksum(&[0x00, 0x00, 0x00, 0x00]));
        assert_eq!(0x01, checksum(&[0x01, 0x00, 0x00, 0x00]));
        assert_eq!(0xde, checksum(&[0x16, 0xfa, 0xbb, 0x13]));
    }

    #[test]
    fn test_validate_checksum() {
        assert_eq!(true, validate_checksum(&[0x00, 0x00, 0x00, 0x00]));
        assert_eq!(true, validate_checksum(&[0x01, 0x00, 0x00, 0x01]));
        assert_eq!(false, validate_checksum(&[0xff, 0xfa, 0xbb, 0x13]));
        assert_eq!(false, validate_checksum(&[0x16, 0xfa, 0xbb, 0xff]));
    }

    #[test]
    fn test_serialization() {
        let k30 = Protocol::k30();
        let (request, cb) = k30.co2_ppm();
        assert_eq!(request, [0x22, 0x00, 0x08, 0x2A]);
        assert_eq!(Err(Error::ChecksumMismatch), cb(&[0x10, 0x00, 0x00, 0x00]));
        assert_eq!(Err(Error::Incomplete), cb(&[0x00, 0x00, 0x00, 0x00]));
        assert_eq!(Ok(4660), cb(&[0x01, 0x12, 0x34, 0x47]));
    }
}
