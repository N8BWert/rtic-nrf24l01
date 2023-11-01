//!
//! Implementation of the nRF24L01+ Module utilizing rtic monotonics
//! non-blocking delay whenever possible
//! 

#![no_std]

#![feature(impl_trait_projections)]

use core::marker::PhantomData;

use config::RegisterValue;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::spi::{Transfer, Write};
#[cfg(feature = "blocking")]
use embedded_hal::blocking::delay::DelayUs;

pub mod command;
use command::Command;

pub mod state;
use state::State;

pub mod config;
use config::{Configuration, RegisterMask};

pub mod error;
use error::Error;

pub mod register;
use register::{StatusRegister, ContainsStatus, ConfigRegister, FifoStatusRegister};

#[cfg(feature = "async")]
use rtic_monotonics::systick::*;
#[cfg(feature = "async")]
use rtic_monotonics::systick::ExtU32;

const MAX_FIFO_SIZE: usize = 32 * 3;
const MAX_PAYLOAD_SIZE: usize = 32;

/// NRF24L01 Driver Implementation with Blocking Delay
#[cfg(feature = "blocking")]
pub struct NRF24L01<
    'a,
    GPIOE,
    SPIE,
    CSN: OutputPin<Error = GPIOE>,
    CE: OutputPin<Error = GPIOE>,
    SPI: Transfer<u8, Error = SPIE> + Write<u8, Error = SPIE>,
    DELAY: DelayUs<u32>> {
    csn: Option<CSN>,
    ce: CE,
    state: State,
    configuration: Configuration<'a>,
    phantom: PhantomData<(SPI, DELAY)>,
}

#[cfg(feature = "blocking")]
impl<'a, GPIOE, SPIE, CSN, CE, SPI, DELAY> NRF24L01<'a, GPIOE, SPIE, CSN, CE, SPI, DELAY> where
    CSN: OutputPin<Error = GPIOE>,
    CE: OutputPin<Error = GPIOE>,
    SPI: Transfer<u8, Error = SPIE> + Write<u8, Error = SPIE>,
    DELAY: DelayUs<u32> {
    pub fn new(csn: Option<CSN>, ce: CE, config: Configuration<'a>, spi: &mut SPI, delay: &mut DELAY) -> Result<Self, Error<GPIOE, SPIE>> {
        // Wait for power on reset
        delay.delay_us(100_000u32);

        let mut driver = Self {
            csn,
            ce,
            state: State::PowerDown,
            configuration: config,
            phantom: PhantomData,
        };

        let mut config = driver.read_config(spi)?;

        if !config.contains_status(ConfigRegister::PowerUp) {
            config |= ConfigRegister::PowerUp as u8;
            driver.write_register(0x00, &[config], spi)?;
            delay.delay_us(1_500u32);
        }

        driver.state = State::Standby1;

        driver.write_full_config(spi)?;

        Ok(driver)
    }

    pub fn read_register(&mut self, register: u8, buffer: &mut [u8], spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        match register {
            // 1 Byte Registers
            0x00..=0x09 | 0x11..=0x17 | 0x1C..=0x1D => {
                if buffer.len() != 1 {
                    return Err(Error::InvalidBufferSize);
                }

                let mut status = [Command::ReadRegister(register).opcode(), 0x00];
                self.safe_transfer_spi(spi, &mut status)?;

                buffer[0] = status[1];
                Ok(status[0])
            },
            // Address Registers (5 Byte Maximum)
            0x0A..=0x10 => {
                if buffer.len() != 5 {
                    return Err(Error::InvalidBufferSize);
                }

                let mut status = [0u8; 6];
                status[0] = Command::ReadRegister(register).opcode();
                self.safe_transfer_spi(spi, &mut status)?;

                buffer[..].clone_from_slice(&status[1..]);
                Ok(status[0])
            },
            _ => return Err(Error::UnknownRegister),
        }
    }

    pub fn write_register(&mut self, register: u8, data: &[u8], spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        match register {
            // 1 Byte Registers
            0x00..=0x09 | 0x11..=0x17 | 0x1C..=0x1D => {
                if data.len() != 1 {
                    return Err(Error::InvalidRegisterBufferSize(register));
                }

                let mut status = [Command::WriteRegister(register).opcode(), data[0]];
                self.safe_transfer_spi(spi, &mut status)?;

                Ok(status[0])
            },
            // Address Registers (5 Byte Maximum)
            0x0A..=0x10 => {
                if data.len() != self.configuration.address_width.as_u8() as usize {
                    return Err(Error::InvalidAddressBufferSize);
                }

                let mut status = [0u8; 5];
                status[0] = Command::WriteRegister(register).opcode();
                status[1..].copy_from_slice(data);
                self.safe_transfer_spi(spi, &mut status)?;

                Ok(status[0])
            },
            _ => return Err(Error::UnknownRegister),
        }
    }

    pub fn read_payload(&mut self, buffer: &mut [u8], spi: &mut SPI, delay: &mut DELAY) -> Result<u8, Error<GPIOE, SPIE>> {
        if self.state != State::Rx {
            self.to_state(State::Rx, spi, delay)?;
        }

        if buffer.len() > MAX_FIFO_SIZE {
            return Err(Error::InvalidBufferSize);
        }

        let mut buffer_idx = 0;
        let mut iteration = 0;
        let mut status = 0u8;
        while buffer.len() - buffer_idx > 0 {
            if iteration == 3 {
                break;
            }

            let payload_length = self.read_payload_length(spi)?;
            if buffer_idx + payload_length as usize > buffer.len() {
                return Err(Error::InvalidBufferSize);
            }
            
            let mut status_buffer = [0u8; 33];
            status_buffer[0] = Command::ReadRxPayload.opcode();

            self.safe_transfer_spi(spi, &mut status_buffer)?;

            buffer[buffer_idx..].clone_from_slice(&status_buffer[1..=(1+payload_length as usize)]);

            buffer_idx += payload_length as usize;
            iteration += 1;
            status = status_buffer[0];
        }

        Ok(status)
    }

    pub fn write_payload(&mut self, payload: &[u8], spi: &mut SPI, delay: &mut DELAY) -> Result<u8, Error<GPIOE, SPIE>> {
        if self.state != State::Tx {
            self.to_state(State::Tx, spi, delay)?;
        }

        let mut iteration = 0;
        let mut status = 0u8;

        while iteration * 32 < payload.len() {
            if iteration % 3 == 0 {
                // Wait for FIFO Queue to clear
                match self.configuration.data_rate {
                    DataRate::R250kb => delay.delay_us(4_000u32),
                    DataRate::R1Mb | DataRate::R2Mb => delay.delay_us(1_000u32),
                }

                let mut status = self.read_status(spi)?;

                while status.contains_status(StatusRegister::TxFull) {
                    delay.delay_us(1_000u32);
                    
                    status = self.read_status(spi)?;
                }
            }

            let mut status_buffer = [0u8; 33];
            status_buffer[0] = Command::WriteTxPayload.opcode();
            status_buffer[1..].copy_from_slice(&payload[(iteration * 32)..]);

            self.safe_transfer_spi(spi, &mut status_buffer)?;

            status = status_buffer[0];
            iteration += 1;
        }

        Ok(status)
    }

    pub fn write_payload_no_ack(&mut self, payload: &[u8], spi: &mut SPI, delay: &mut DELAY) -> Result<u8, Error<GPIOE, SPIE>> {
        if self.state != State::Tx {
            self.to_state(State::Tx, spi, delay)?;
        }

        let mut iteration = 0;
        let mut status = 0u8;

        while iteration * 32 < payload.len() {
            if iteration % 3 == 0 {
                // Wait for FIFO Queue to clear
                match self.configuration.data_rate {
                    DataRate::R250kb => delay.delay_us(4_000u32),
                    DataRate::R1Mb | DataRate::R2Mb => delay.delay_us(1_000u32),
                }

                let mut status = self.read_status(spi)?;

                while status.contains_status(StatusRegister::TxFull) {
                    delay.delay_us(1_000u32);

                    status = self.read_status(spi)?;
                }
            }

            let mut status_buffer = [0u8; 33];
            status_buffer[0] = Command::WriteTxPayload.opcode();
            status_buffer[1..].copy_from_slice(&payload[(iteration * 32)..]);

            self.safe_transfer_spi(spi, &mut status_buffer)?;

            status = status_buffer[0];
            iteration += 1;
        }

        Ok(status)
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

    pub fn read_config(&mut self, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        let mut config = [Command::ReadRegister(0x00).opcode(), 0x00];
        self.safe_transfer_spi(spi, &mut config)?;
        Ok(config[1])
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

    pub fn to_state(&mut self, state: State, spi: &mut SPI, delay: &mut DELAY) -> Result<(), Error<GPIOE, SPIE>> {
        if state == self.state {
            return Ok(());
        }

        let mut config = self.read_config(spi)?;

        // There are a few filled in cases that could be done with recursion, but recursion with async functions is
        // no bueno
        match (self.state, state) {
            (State::PowerDown, State::Standby1) => {
                if !config.contains_status(ConfigRegister::PowerUp) {
                    config |= ConfigRegister::PowerUp as u8;
                    self.write_register(0x00, &[config], spi)?;
                    delay.delay_us(1_500u32);
                }

                self.state = State::Standby1;
            },
            (State::Standby1, State::Rx) => {
                config |= ConfigRegister::RxTxControl as u8;
                self.ce.set_high().map_err(Error::GpioError)?;
                self.write_register(0x00, &[config], spi)?;
                delay.delay_us(130u32);

                self.state = State::Rx;
            },
            (State::Standby1, State::Tx) => {
                config &= !(ConfigRegister::RxTxControl as u8);
                self.ce.set_high().map_err(Error::GpioError)?;
                self.write_register(0x00, &[config], spi)?;
                delay.delay_us(140u32);

                self.state = State::Tx;
            },
            (State::Tx, State::Standby1) | (State::Rx, State::Standby1) => {
                self.ce.set_low().map_err(Error::GpioError)?;

                self.state = State::Standby1;
            },
            (State::Standby1, State::PowerDown) => {
                config &= !(ConfigRegister::PowerUp as u8);
                self.write_register(0x00, &[config], spi)?;
                
                self.state = State::PowerDown;
            },
            (State::PowerDown, State::Rx) => {
                if !config.contains_status(ConfigRegister::PowerUp) {
                    config |= ConfigRegister::PowerUp as u8;
                    self.write_register(0x00, &[config], spi)?;
                    delay.delay_us(1_500u32);
                }

                self.state = State::Standby1;

                config |= ConfigRegister::RxTxControl as u8;
                self.ce.set_high().map_err(Error::GpioError)?;
                self.write_register(0x00, &[config], spi)?;
                delay.delay_us(130u32);

                self.state = State::Rx;
            },
            (State::PowerDown, State::Tx) => {
                if !config.contains_status(ConfigRegister::PowerUp) {
                    config |= ConfigRegister::PowerUp as u8;
                    self.write_register(0x00, &[config], spi)?;
                    delay.delay_us(1_500u32);
                }

                self.state = State::Standby1;

                config &= !(ConfigRegister::RxTxControl as u8);
                self.ce.set_high().map_err(Error::GpioError)?;
                self.write_register(0x00, &[config], spi)?;
                delay.delay_us(140u32);

                self.state = State::Tx;
            },
            (State::Rx, State::Tx) => {
                self.ce.set_low().map_err(Error::GpioError)?;

                self.state = State::Standby1;

                config &= !(ConfigRegister::RxTxControl as u8);
                self.ce.set_high().map_err(Error::GpioError)?;
                self.write_register(0x00, &[config], spi)?;
                delay.delay_us(140u32);

                self.state = State::Tx;
            },
            (State::Tx, State::Rx) => {
                self.ce.set_low().map_err(Error::GpioError)?;

                self.state = State::Standby1;

                config |= ConfigRegister::RxTxControl as u8;
                self.ce.set_high().map_err(Error::GpioError)?;
                self.write_register(0x00, &[config], spi)?;
                delay.delay_us(130u32);

                self.state = State::Rx;
            },
            _ => (),
        }

        Ok(())
    }

    fn write_full_config(&mut self, spi: &mut SPI) -> Result<(), Error<GPIOE, SPIE>> {
        // Write first 7 configs
        for register in 0x00..=0x06 {
            let mut config = [0u8];

            self.read_register(register, &mut config, spi)?;
            config[0] &= self.configuration.register_mask(register);
            config[0] |= self.configuration.register_value(register);
            self.write_register(register, &config, spi)?;

            let mut found_config = [0u8];
            self.read_register(register, &mut found_config, spi)?;

            if found_config[0] != config[0] {
                return Err(Error::UnableToConfigureRegister(register));
            }
        }

        // Write Register and Feature Registers
        for register in 0x1C..=0x1D {
            if let Some(pipe_config) = self.configuration.pipe_configs[0] {
                self.write_register(register, pipe_config.address, spi)?;

                let mut found_config = [0u8; 5];
                self.read_register(register, &mut found_config, spi)?;

                for i in 0..pipe_config.address.len() {
                    if pipe_config.address[i] != found_config[i] {
                        return Err(Error::UnableToConfigureRegister(register));
                    }
                }
            }
        }

        // Write Address Registers
        for register in 0x0A..=010 {
            let mut config = [0u8];

            self.read_register(register, &mut config, spi)?;
            config[0] &= self.configuration.register_mask(register);
            config[0] |= self.configuration.register_value(register);
            self.write_register(register, &config, spi)?;

            let mut found_config = [0u8];
            self.read_register(register, &mut found_config, spi)?;

            if found_config[0] != config[0] {
                return Err(Error::UnableToConfigureRegister(register));
            }
        }

        Ok(())
    }
}

/// NRF24L01 Driver Implementation With Rtic Monotnic Delays
#[cfg(feature = "async")]
pub struct NRF24L01<
    'a,
    GPIOE,
    SPIE,
    CSN: OutputPin<Error = GPIOE>,
    CE: OutputPin<Error = GPIOE>,
    SPI: Transfer<u8, Error = SPIE> + Write<u8, Error = SPIE>> {
    csn: Option<CSN>,
    ce: CE,
    state: State,
    configuration: Configuration<'a>,
    phantom: PhantomData<SPI>,
}

#[cfg(feature = "async")]
impl<'a, GPIOE, SPIE, CSN, CE, SPI> NRF24L01<'a, GPIOE, SPIE, CSN, CE, SPI> where
    CSN: OutputPin<Error = GPIOE>,
    CE: OutputPin<Error = GPIOE>,
    SPI: Transfer<u8, Error = SPIE> + Write<u8, Error = SPIE> {
    pub async fn new(csn: Option<CSN>, ce: CE, config: Configuration<'a>, spi: &mut SPI) -> Result<Self, Error<GPIOE, SPIE>> {
        // Wait for power on reset
        Systick::delay(100u32.millis()).await;

        let mut driver = Self {
            csn,
            ce,
            state: State::PowerDown,
            configuration: config,
            phantom: PhantomData,
        };

        let mut config = driver.read_config(spi)?;

        if !config.contains_status(ConfigRegister::PowerUp) {
            config |= ConfigRegister::PowerUp as u8;
            driver.write_register(0x00, &[config], spi)?;
            Systick::delay(1_500.micros()).await;
        }

        driver.state = State::Standby1;

        driver.write_full_config(spi)?;

        Ok(driver)
    }

    pub fn read_register(&mut self, register: u8, buffer: &mut [u8], spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        match register {
            // 1 Byte Registers
            0x00..=0x09 | 0x0B..=0x0F | 0x11..=0x17 | 0x1C..=0x1D => {
                if buffer.len() != 1 {
                    return Err(Error::InvalidBufferSize);
                }

                let mut status = [Command::ReadRegister(register).opcode(), 0x00];
                self.safe_transfer_spi(spi, &mut status)?;

                buffer[0] = status[1];
                Ok(status[0])
            },
            // Address Registers (5 Byte Maximum)
            0x0A | 0x10 => {
                if buffer.len() != 5 {
                    return Err(Error::InvalidBufferSize);
                }

                let mut status = [0u8; 6];
                status[0] = Command::ReadRegister(register).opcode();
                self.safe_transfer_spi(spi, &mut status)?;

                buffer[..].clone_from_slice(&status[1..]);
                Ok(status[0])
            },
            _ => return Err(Error::UnknownRegister),
        }
    }

    pub fn write_register(&mut self, register: u8, data: &[u8], spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        match register {
            // 1 Byte Registers
            0x00..=0x09 | 0x0B..=0x0F | 0x1C..=0x1D => {
                if data.len() != 1 {
                    return Err(Error::InvalidRegisterBufferSize(register));
                }

                let mut status = [Command::WriteRegister(register).opcode(), data[0]];
                self.safe_transfer_spi(spi, &mut status)?;

                Ok(status[0])
            },
            // Address Registers (5 Byte Maximum)
            0x0A | 0x10 => {
                if data.len() != self.configuration.address_width.as_u8() as usize {
                    return Err(Error::InvalidAddressBufferSize);
                }

                let mut status = [0u8; 6];
                status[0] = Command::WriteRegister(register).opcode();
                status[1..1+data.len()].copy_from_slice(data);
                self.safe_transfer_spi(spi, &mut status)?;

                Ok(status[0])
            },
            _ => return Err(Error::UnknownRegister),
        }
    }

    /// Read a piece of data from the Rx FIFO of max length 32 returning (data.len(), rx_pipe)
    pub async fn read_data(&mut self, data_buffer: &mut [u8], spi: &mut SPI) -> Result<(u8, u8), Error<GPIOE, SPIE>> {
        // Convert to Rx State
        if self.state != State::Rx {
            self.to_state(State::Rx, spi).await?;
        }

        // Make sure the data_buffer is not too large
        if data_buffer.len() != 32 {
            return Err(Error::InvalidBufferSize);
        }

        // Read the incoming data pipe number
        let status = self.read_status(spi)?;
        let pipe_num = status & 0b0000_1110;

        // Read the incoming data length
        let data_length = self.read_payload_length(spi)?;

        // Read the rx data
        let mut rx_buffer = [0u8; 33];
        rx_buffer[0] = Command::ReadRxPayload.opcode();
        self.safe_transfer_spi(spi, &mut rx_buffer)?;

        // Transfer the data into the buffer
        data_buffer[..].copy_from_slice(&rx_buffer[1..]);

        Ok((data_length, pipe_num))
    }

    /// Read data from the device from a given pipe until that pipe is no longer the next incoming data.
    pub async fn read_from_pipe(&mut self, pipe: u8, buffer: &mut [u8], spi: &mut SPI) -> Result<usize, Error<GPIOE, SPIE>> {
        // Convert to Rx State
        if self.state != State::Rx {
            self.to_state(State::Rx, spi).await?;
        }

        // Keep track of where in the buffer we're writing to
        let mut buffer_idx = 0usize;

        let status = self.read_status(spi)?;
        let mut pipe_num = status & 0b0000_1110;
        while pipe_num == pipe {
            // Read the incoming data length
            let data_length = self.read_payload_length(spi)?;

            // Check whether we can actually read the next piece of data
            if data_length as usize + buffer_idx >= buffer.len() {
                return Ok(buffer.len() - 1 - buffer_idx);
            }

            // Read the rx data
            let mut rx_buffer = [0u8; 33];
            rx_buffer[0] = Command::ReadRxPayload.opcode();
            self.safe_transfer_spi(spi, &mut rx_buffer)?;

            // Transfer the data into the buffer
            buffer[buffer_idx..(buffer_idx + data_length as usize)].copy_from_slice(&rx_buffer[1..(1+data_length as usize)]);

            // Increment the buffer_idx
            buffer_idx += data_length as usize;

            // Update the next fx pipe number
            let status = self.read_status(spi)?;
            pipe_num = status & 0b0000_1110;
        }

        Ok(buffer_idx)
    }

    /// Send a payload of up to 32 bytes with or without auto acknowledgement
    pub async fn send_data(&mut self, payload: &[u8], auto_ack: bool, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        // Convert to Tx State
        if self.state != State::Tx {
            self.to_state(State::Tx, spi).await?;
        }

        // Make sure the data is not too large
        if payload.len() > 32 {
            return Err(Error::InvalidDataSize);
        }

        // Wait until the tx FIFO is not full
        let mut fifo_status = self.read_fifo_status(spi)?;
        while fifo_status.contains_status(FifoStatusRegister::TxFull) {
            // Wait for the fifo to not be empty
            // TODO: Optimize this value
            Systick::delay(10u32.millis()).await;

            fifo_status = self.read_fifo_status(spi)?;
        }

        // Write the correct command
        let mut send_buffer = [0u8; 33];
        if auto_ack {
            send_buffer[0] = Command::WriteTxPayload.opcode();
        } else {
            send_buffer[0] = Command::WriteTxNoAck.opcode();
        }

        // Copy in the payload to send
        send_buffer[1..(1+payload.len())].copy_from_slice(payload);

        // Send the payload
        self.safe_transfer_spi(spi, &mut send_buffer[..payload.len()])?;

        // Return the device status
        Ok(send_buffer[0])
    }

    // Send a payload of longer than 32 Bytes
    pub async fn send_payloads(&mut self, payload: &[u8], auto_ack: bool, spi: &mut SPI) -> Result<(), Error<GPIOE, SPIE>> {
        // Convert to Tx State
        if self.state != State::Tx {
            self.to_state(State::Tx, spi).await?;
        }

        // Wait until the tx FIFO is not full
        let mut fifo_status = self.read_fifo_status(spi)?;
        while fifo_status.contains_status(FifoStatusRegister::TxFull) {
            // Wait for the fifo to not be empty
            // TODO: Optimize this value
            Systick::delay(10u32.millis()).await;

            fifo_status = self.read_fifo_status(spi)?;
        }

        // Send (payload.len() / 32) 32 Byte payloads and 1 (payload.len() % 32) payload
        for payload_num in 0..(payload.len() / 32) {
            let mut send_buffer = [0u8; 33];
            if auto_ack {
                send_buffer[0] = Command::WriteTxPayload.opcode();
            } else {
                send_buffer[0] = Command::WriteTxNoAck.opcode();
            }

            // Copy in the payload to send
            send_buffer[1..].copy_from_slice(&payload[(payload_num * 32)..((payload_num + 1) * 32)]);
            self.safe_transfer_spi(spi, &mut send_buffer)?;

            // Wait until the tx FIFO is not full
            fifo_status = self.read_fifo_status(spi)?;
            while fifo_status.contains_status(FifoStatusRegister::TxFull) {
                // Wait for the fifo to not be empty
                // TODO: Optimize this value
                Systick::delay(10u32.millis()).await;

                fifo_status = self.read_fifo_status(spi)?;
            }
        }

        // Send (payload.len() % 32) payload
        let payload_len = payload.len() % 32;
        if payload_len == 0 {
            return Ok(());
        }

        let mut send_buffer = [0u8; 33];
        if auto_ack {
            send_buffer[0] = Command::WriteTxPayload.opcode();
        } else {
            send_buffer[0] = Command::WriteTxNoAck.opcode();
        }

        // Copy in the payload to send
        send_buffer[1..(1+payload_len)].copy_from_slice(&payload[(payload.len()-1-payload_len)..(payload.len()-1)]);
        self.safe_transfer_spi(spi, &mut send_buffer[0..(1+payload_len)])?;

        Ok(())
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

    pub fn read_config(&mut self, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        let mut config = [Command::ReadRegister(0x00).opcode(), 0x00];
        self.safe_transfer_spi(spi, &mut config)?;
        Ok(config[1])
    }

    pub fn read_fifo_status(&mut self, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        let mut status = [Command::ReadRegister(0x17).opcode(), 0x00];
        self.safe_transfer_spi(spi, &mut status)?;
        Ok(status[1])
    }

    pub fn clear_interrupt(&mut self, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        self.write_register(0x07, &[0b1011_0000], spi)?;
        self.read_status(spi)
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

    pub async fn to_state(&mut self, state: State, spi: &mut SPI) -> Result<(), Error<GPIOE, SPIE>> {
        if state == self.state {
            return Ok(());
        }

        let mut config = self.read_config(spi)?;

        // There are a few filled in cases that could be done with recursion, but recursion with async functions is
        // no bueno
        match (self.state, state) {
            (State::PowerDown, State::Standby1) => {
                if !config.contains_status(ConfigRegister::PowerUp) {
                    config |= ConfigRegister::PowerUp as u8;
                    self.write_register(0x00, &[config], spi)?;
                    Systick::delay(1500u32.micros()).await;
                }

                self.state = State::Standby1;
            },
            (State::Standby1, State::Rx) => {
                config |= ConfigRegister::RxTxControl as u8;
                self.ce.set_high().map_err(Error::GpioError)?;
                self.write_register(0x00, &[config], spi)?;
                Systick::delay(130u32.micros()).await;

                self.state = State::Rx;
            },
            (State::Standby1, State::Tx) => {
                config &= !(ConfigRegister::RxTxControl as u8);
                self.ce.set_high().map_err(Error::GpioError)?;
                self.write_register(0x00, &[config], spi)?;
                Systick::delay(140u32.micros()).await;

                self.state = State::Tx;
            },
            (State::Tx, State::Standby1) | (State::Rx, State::Standby1) => {
                self.ce.set_low().map_err(Error::GpioError)?;

                self.state = State::Standby1;
            },
            (State::Standby1, State::PowerDown) => {
                config &= !(ConfigRegister::PowerUp as u8);
                self.write_register(0x00, &[config], spi)?;
                
                self.state = State::PowerDown;
            },
            (State::PowerDown, State::Rx) => {
                if !config.contains_status(ConfigRegister::PowerUp) {
                    config |= ConfigRegister::PowerUp as u8;
                    self.write_register(0x00, &[config], spi)?;
                    Systick::delay(1500u32.micros()).await;
                }

                self.state = State::Standby1;

                config |= ConfigRegister::RxTxControl as u8;
                self.ce.set_high().map_err(Error::GpioError)?;
                self.write_register(0x00, &[config], spi)?;
                Systick::delay(130u32.micros()).await;

                self.state = State::Rx;
            },
            (State::PowerDown, State::Tx) => {
                if !config.contains_status(ConfigRegister::PowerUp) {
                    config |= ConfigRegister::PowerUp as u8;
                    self.write_register(0x00, &[config], spi)?;
                    Systick::delay(1500u32.micros()).await;
                }

                self.state = State::Standby1;

                config &= !(ConfigRegister::RxTxControl as u8);
                self.ce.set_high().map_err(Error::GpioError)?;
                self.write_register(0x00, &[config], spi)?;
                Systick::delay(140u32.micros()).await;

                self.state = State::Tx;
            },
            (State::Rx, State::Tx) => {
                self.ce.set_low().map_err(Error::GpioError)?;

                self.state = State::Standby1;

                config &= !(ConfigRegister::RxTxControl as u8);
                self.ce.set_high().map_err(Error::GpioError)?;
                self.write_register(0x00, &[config], spi)?;
                Systick::delay(140u32.micros()).await;

                self.state = State::Tx;
            },
            (State::Tx, State::Rx) => {
                self.ce.set_low().map_err(Error::GpioError)?;

                self.state = State::Standby1;

                config |= ConfigRegister::RxTxControl as u8;
                self.ce.set_high().map_err(Error::GpioError)?;
                self.write_register(0x00, &[config], spi)?;
                Systick::delay(130u32.micros()).await;

                self.state = State::Rx;
            },
            _ => (),
        }

        Ok(())
    }

    fn write_full_config(&mut self, spi: &mut SPI) -> Result<(), Error<GPIOE, SPIE>> {
        // Write first 7 configs
        for register in 0x00..=0x06 {
            let mut config = [0u8];

            self.read_register(register, &mut config, spi)?;
            config[0] &= self.configuration.register_mask(register);
            config[0] |= self.configuration.register_value(register);
            self.write_register(register, &config, spi)?;

            let mut found_config = [0u8];
            self.read_register(register, &mut found_config, spi)?;

            if found_config[0] != config[0] {
                return Err(Error::UnableToConfigureRegister(register));
            }
        }

        // Write Base Register and Feature Registers
        let base_config = self.configuration.pipe_configs[0].unwrap();
        self.write_register(0x0A, base_config.address, spi)?;

        let mut found_config = [0u8; 5];
        self.read_register(0x0A, &mut found_config, spi)?;

        for i in 0..base_config.address.len() {
            if base_config.address[i] != found_config[i] {
                return Err(Error::UnableToConfigureRegister(0x0A));
            }
        }

        // Write Other Pipe Registers
        let mut pipe_index = 1;
        for register in 0x0B..=0x0F {
            if let Some(pipe_config) = self.configuration.pipe_configs[pipe_index] {
                self.write_register(register, &[pipe_config.address[0]], spi)?;

                let mut found_config = [0u8];
                self.read_register(register, &mut found_config, spi)?;

                if found_config[0] != pipe_config.address[0] {
                    return Err(Error::UnableToConfigureRegister(register))?;
                }
            } else {
                break;
            }

            pipe_index += 1;
        }

        // Write Transmit Register
        self.write_register(0x10, self.configuration.tx_address, spi)?;

        let mut found_config = [0u8; 5];
        self.read_register(0x10, &mut found_config, spi)?;

        for i in 0..self.configuration.tx_address.len() {
            if self.configuration.tx_address[i] != found_config[i] {
                return Err(Error::UnableToConfigureRegister(0x10));
            }
        }

        // Write Address Registers
        for register in 0x1C..=0x1D {
            let mut config = [0u8];

            self.read_register(register, &mut config, spi)?;
            config[0] &= self.configuration.register_mask(register);
            config[0] |= self.configuration.register_value(register);
            self.write_register(register, &config, spi)?;

            let mut found_config = [0u8];
            self.read_register(register, &mut found_config, spi)?;

            if found_config[0] != config[0] {
                return Err(Error::UnableToConfigureRegister(register));
            }
        }

        Ok(())
    }
}