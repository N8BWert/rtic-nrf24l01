//!
//! Error for the NRF24L01 Module
//! 

use crate::register::Register;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RadioError {
    InvalidAddressLength,
    InvalidPayloadLength,
    NoPacketReady,
    UnableToWriteToRegister{register: Register, expected: u8, found: u8},
}