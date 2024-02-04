//!
//! Error for the NRF24L01 Module
//! 

use crate::register::Register;

use defmt::Format;

#[derive(Clone, Copy, PartialEq, Eq, Debug, Format)]
pub enum RadioError {
    InvalidAddressLength,
    InvalidPayloadLength,
    NoPacketReady,
    SendTimeout,
    BadConfiguration,
    UnableToWriteToRegister{register: Register, expected: u8, found: u8},
}