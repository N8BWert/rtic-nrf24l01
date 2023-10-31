//!
//! Implementation of the nRF24L01+ Module utilizing rtic monotonics
//! non-blocking delay whenever possible
//! 

#![no_std]

use core::marker::PhantomData;

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::spi::{Transfer, Write};

pub mod command;
use command::Command;

pub mod state;
use state::State;

pub mod config;
use config::Configuration;

pub mod error;
use error::Error;

use rtic_monotonics::systick::*;
use rtic_monotonics::systick::ExtU32;

pub struct NRF24L01<
    GPIOE,
    SPIE,
    CSN: OutputPin<Error = GPIOE>,
    CE: OutputPin<Error = GPIOE>,
    SPI: Transfer<u8, Error = SPIE> + Write<u8, Error = SPIE>> {
    csn: Option<CSN>,
    ce: CE,
    state: State,
    phantom: PhantomData<SPI>,
}

impl<GPIOE, SPIE, CSN, CE, SPI> NRF24L01<GPIOE, SPIE, CSN, CE, SPI> where
    CSN: OutputPin<Error = GPIOE>,
    CE: OutputPin<Error = GPIOE>,
    SPI: Transfer<u8, Error = SPIE> + Write<u8, Error = SPIE> {
    pub fn new(csn: Option<CSN>, ce: CE, spi: SPI, config: Configuration) -> Result<Self, Error<GPIOE, SPIE>> {
        todo!()
    }

    pub fn read_register(&mut self, register: u8, buffer: &mut [u8], spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        todo!()
    }

    pub fn write_register(&mut self, register: u8, data: &[u8], spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        todo!()
    }

    pub fn read_payload(&mut self, buffer: &mut [u8], spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        todo!()
    }

    pub fn write_payload(&mut self, payload: &[u8], spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        todo!()
    }

    pub fn write_payload_no_ack(&mut self, payload: &[u8], spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        todo!()
    }

    pub fn flush_tx(&mut self, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        let mut status = [Command::FlushTx.opcode()];
        self.safe_transfer_spi(spi, &mut status)?;
        Ok(status[0])
    }

    pub fn flush_rx(&mut self, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        let mut status = [Command::FlushRx.opcode()];
        self.safe_transfer_spi(spi, &mut status)?;
        Ok(status[0])
    }

    pub fn retransmit_payload(&mut self, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        let mut status = [Command::ReuseTxPayload.opcode()];
        self.safe_transfer_spi(spi, &mut status)?;
        Ok(status[0])
    }

    pub fn read_payload_length(&mut self, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        let mut data: [u8; 2] = [0u8; 2];
        data[0] = Command::ReadRxPayloadWidth.opcode();
        self.safe_transfer_spi(spi, &mut data)?;
        Ok(data[1])
    }

    pub fn set_ack_payload(&mut self, pipe: u8, ack_payload: &[u8], spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        // Handle invalid cases
        if pipe > 5 {
            return Err(Error::InvalidPipeId);
        } else if ack_payload.len() > 32 {
            return Err(Error::TooLargeAckPayload);
        }

        // Write data in form [opcode, data0, data1, ... data31]
        let mut write_data: [u8; 33] = [0u8; 33];
        write_data[0] = Command::WriteAcknowledgePayload(pipe).opcode();
        write_data[1..].clone_from_slice(ack_payload);

        self.safe_transfer_spi(spi, &mut write_data)?;

        Ok(write_data[0])
    }

    pub fn read_status(&mut self, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        let mut status = [Command::Nop.opcode()];
        self.safe_transfer_spi(spi, &mut status)?;
        Ok(status[0])
    }

    fn safe_transfer_spi(&mut self, spi: &mut SPI, buffer: &mut [u8]) -> Result<(), Error<GPIOE, SPIE>> {
        match self.csn.as_mut() {
            Some(csn) => {
                csn.set_low().map_err(Error::GpioError)?;
                let transfer_result = spi.transfer(buffer);
                let gpio_error = csn.set_high();

                match (transfer_result, gpio_error) {
                    (Err(err), Ok(_)) => Err(Error::SpiError(err)),
                    (Ok(_), Err(err)) => Err(Error::GpioError(err)),
                    (Err(spi_err), Err(gpio_err)) => Err(Error::GpioSpiError((gpio_err, spi_err))),
                    _ => Ok(()),
                }
            },
            None => {
                spi.transfer(buffer).map_err(Error::SpiError)?;
                Ok(())
            }
        }
    }
}

