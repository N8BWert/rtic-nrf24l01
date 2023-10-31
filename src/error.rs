//!
//! Error for the NRF24L01 Module
//! 

pub enum Error<GPIOE, SPIE> {
    InvalidPipeId,
    TooLargeAckPayload,
    GpioError(GPIOE),
    SpiError(SPIE),
    GpioSpiError((GPIOE, SPIE)),
}