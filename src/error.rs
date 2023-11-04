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
    InvalidRfChannel,
    InvalidRetransmitDelay,
    InvalidRetransmitCount,
    UnableToConfigureRegister(u8, u8, u8),
    GpioError(GPIOE),
    SpiError(SPIE),
    GpioSpiError((GPIOE, SPIE)),
}