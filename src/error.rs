//!
//! Error for the NRF24L01 Module
//! 

use crate::register::Register;

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
pub enum RadioError {
    InvalidBufferLength,
    InvalidRfChannel,
    InvalidPayloadSize,
    // (register, expected, found)
    UnableToConfigureRegister(Register, u8, u8),
}

pub enum Error<GPIOE, SPIE> {
    InvalidPipeId,
    TooLargeAckPayload,
    InvalidBufferSize,
    InvalidRegisterBufferSize(u8),
    InvalidAddressBufferSize,
    UnknownRegister,
    NoDataToReceive,
    InvalidDataSize,
    InvalidRfChannel,
    InvalidRetransmitDelay,
    InvalidRetransmitCount,
    SwitchToTx,
    SwitchToRx,
    UnableToConfigureRegister(u8, u8, u8),
    GpioError(GPIOE),
    SpiError(SPIE),
    GpioSpiError((GPIOE, SPIE)),
}