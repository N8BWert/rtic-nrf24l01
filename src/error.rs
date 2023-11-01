//!
//! Error for the NRF24L01 Module
//! 

pub enum Error<GPIOE, SPIE> {
    InvalidPipeId,
    TooLargeAckPayload,
    InvalidBufferSize,
    InvalidRegisterBufferSize(u8),
    InvalidAddressBufferSize,
    UnknownRegister,
    NoDataToReceive,
    InvalidDataSize,
    UnableToConfigureRegister(u8),
    GpioError(GPIOE),
    SpiError(SPIE),
    GpioSpiError((GPIOE, SPIE)),
}