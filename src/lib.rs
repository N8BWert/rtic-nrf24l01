//!
//! Implementation of the nRF24L01+ Module utilizing rtic monotonics
//! non-blocking delay whenever possible
//! 

#![no_std]

#[cfg(all(feature = "systick", feature = "rp2040"))]
compile_error!("systick and rp2040 are two ways of doing the same thing and are, therefore, mutually exclusive.  Please choose one or the other");

use core::marker::PhantomData;

use config::power_amplifier::PowerAmplifier;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::blocking::delay::{DelayUs, DelayMs};

pub mod command;
use command::Command;

pub mod state;
use state::State;

pub mod config;
use config::*;

pub mod error;
use error::RadioError;

pub mod register;
use register::Register;

mod bit_mnemonics;

#[cfg(feature = "systick")]
use rtic_monotonics::systick::*;
#[cfg(feature = "systick")]
use rtic_monotonics::systick::ExtU32;

#[cfg(feature = "rp2040")]
use rtic_monotonics::rp2040::*;
#[cfg(feature = "rp2040")]
use rtic_monotonics::rp2040::ExtU64;

const POWERUP_DELAY_US: u32 = 5000;

pub enum Fifo {
    Rx = 0,
    Tx = 1,
}

pub enum FifoStatus {
    NotEmpty = 0,
    Empty = 1,
    Full = 2,
}

pub struct Interrupt {
    tx_ok: bool,
    tx_fail: bool,
    rx_ready: bool,
}
/// NRF24L01 Driver Implementation
pub struct NRF24L01<
    'a,
    GPIOE,
    SPIE,
    CSN: OutputPin<Error = GPIOE>,
    CE: OutputPin<Error = GPIOE>,
    SPI: Transfer<u8, Error = SPIE> + Write<u8, Error = SPIE>,
    DELAY: DelayUs<u32> + DelayMs<u32>> {
    csn: CSN,
    ce: CE,
    state: State,
    configuration: Configuration<'a>,
    dynamic_payload: bool,
    payload_length: u8,
    phantom: PhantomData<(SPI, DELAY)>,
}

impl<'a, GPIOE, SPIE, CSN, CE, SPI, DELAY> NRF24L01<'a, GPIOE, SPIE, CSN, CE, SPI, DELAY> where
    CSN: OutputPin<Error = GPIOE>,
    CE: OutputPin<Error = GPIOE>,
    SPI: Transfer<u8, Error = SPIE> + Write<u8, Error = SPIE>,
    DELAY: DelayUs<u32> + DelayMs<u32> {
    pub fn new(csn: CSN, ce: CE, configuration: Configuration<'a>) -> Self {
        Self {
            csn,
            ce,
            state: State::Uninitialized,
            configuration,
            dynamic_payload: false,
            payload_length: 32u8,
            phantom: PhantomData,
        }
    }

    pub fn begin(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), RadioError> {
        for register in Register::get_writable_byte_registers() {
            if let Some(expected_config) = self.configuration.register_value(register) {
                if register == Register::DynamicPayload {
                    self.enable_dynamic_payloads(spi);
                    delay.delay_ms(5);
                    continue;
                }

                self.write_byte_register(expected_config, register, spi);

                delay.delay_ms(5);

                let found_config = self.read_byte_register(register, spi);

                if found_config != expected_config {
                    return Err(RadioError::UnableToConfigureRegister(register, expected_config, found_config));
                }
            }
        }

        // Write the longer registers
        for (i, register) in [Register::RxAddressP0, Register::RxAddressP1].iter().enumerate() {
            if self.configuration.pipe_configs[i].enabled {
                self.write_register(self.configuration.pipe_configs[i].address, *register, spi)?;

                delay.delay_ms(5);

                let mut found_config = [0u8; 5];
                self.read_register(*register, &mut found_config, spi)?;

                for j in 0..self.configuration.pipe_configs[i].address.len() {
                    if found_config[j] != self.configuration.pipe_configs[i].address[j] {
                        return Err(RadioError::UnableToConfigureRegister(*register, self.configuration.pipe_configs[i].address[j], found_config[j]));
                    }
                }
            }
        }

        // Write the Tx register
        self.write_register(self.configuration.tx_address, Register::TxAddress, spi)?;

        delay.delay_ms(5);

        let mut found_config = [0u8; 5];
        self.read_register(Register::TxAddress, &mut found_config, spi)?;

        for i in 0..self.configuration.tx_address.len() {
            if found_config[i] != self.configuration.tx_address[i] {
                return Err(RadioError::UnableToConfigureRegister(Register::TxAddress, self.configuration.tx_address[i], found_config[i]));
            }
        }

        Ok(())
    }

    pub fn enable_dynamic_payloads(&mut self, spi: &mut SPI) {
        let mut feature = self.read_byte_register(Register::Feature, spi);
        feature |= 1 << 2;
        self.write_byte_register(feature, Register::Feature, spi);
        self.write_byte_register(0b0011_1111, Register::DynamicPayload, spi);
    }

    pub fn disable_dynamic_payloads(&mut self, spi: &mut SPI) {
        self.write_byte_register(0, Register::Feature, spi);
        self.write_byte_register(0, Register::DynamicPayload, spi);
    }

    pub fn write_packet(&mut self, buffer: &[u8], spi: &mut SPI, delay: &mut DELAY) {
        let mut write_buffer = [0u8; 33];
        write_buffer[0] = Command::WriteTxPayload.opcode();
        write_buffer[1..=buffer.len()].copy_from_slice(&buffer[..]);
        self.safe_write_spi(&write_buffer, spi);

        let _ = self.ce.set_high();
        delay.delay_us(11);
        let _ = self.ce.set_low();

        let mut fifo_status = self.read_byte_register(Register::FifoStatus, spi);
        while fifo_status & (1 << 4) != 1 {
            delay.delay_us(100);
            fifo_status = self.read_byte_register(Register::FifoStatus, spi);
        }
    }

    pub fn read_packet(&mut self, buffer: &mut [u8], spi: &mut SPI) -> (u8, u8) {
        let pipe = self.read_byte_register(Register::Status, spi) & (0b111 << 1);
        let data_length = self.write_command(0x00, Command::ReadRxPayloadWidth, spi);
        let mut read_buffer = [0xffu8; 33];
        read_buffer[0] = Command::ReadRxPayload.opcode();
        self.safe_transfer_spi(&mut read_buffer[0..=(data_length as usize)], spi);
        buffer[..].copy_from_slice(&read_buffer[1..=(data_length as usize)]);
        (pipe, data_length)
    }

    pub fn set_rf_channel(&mut self, channel: u8, spi: &mut SPI) -> Result<(), RadioError> {
        if channel >= 125 {
            Err(RadioError::InvalidRfChannel)
        } else {
            self.write_byte_register(channel, Register::RfChannel, spi);
            Ok(())
        }
    }

    pub fn get_rf_channel(&mut self, spi: &mut SPI) -> u8 {
        self.read_byte_register(Register::RfChannel, spi)
    }

    pub fn set_payload_size(&mut self, size: u8, spi: &mut SPI) {
        let payload_size = core::cmp::min(size, 32);
        
        for pipe in 0..6 {
            self.write_byte_register(payload_size, Register::get_payload_length(pipe), spi);
        }

        self.payload_length = payload_size;
    }

    pub fn set_pa_level(&mut self, pa_level: PowerAmplifier, spi: &mut SPI) {
        let mut setup = self.read_byte_register(Register::RfSetup, spi);
        setup &= !(0b0000_0110);
        setup |= pa_level.register_value(Register::RfSetup).unwrap();
        self.write_byte_register(setup, Register::RfSetup, spi);
    }

    pub fn open_writing_pipe(&mut self, address: &[u8], spi: &mut SPI) {
        let _ = self.write_register(address, Register::RxAddressP0, spi);
        let _ = self.write_register(address, Register::TxAddress, spi);
    }

    pub fn open_reading_pipe(&mut self, pipe: u8, address: &[u8], spi: &mut SPI) {
        if pipe < 2 {
            let _ = self.write_register(address, Register::get_rx_address(pipe), spi);
            let mut enabled_addresses = self.read_byte_register(Register::EnabledRxAddresses, spi);
            enabled_addresses |= 1 << pipe;
            self.write_byte_register(enabled_addresses, Register::EnabledRxAddresses, spi);
        }
    }

    pub fn get_payload_size(&mut self, pipe: u8) -> u8 {
        self.payload_length
    }

    pub fn connected(&mut self, spi: &mut SPI) -> bool {
        self.read_byte_register(Register::SetupAddressWidths, spi) == self.configuration.address_width as u8
    }

    pub fn available(&mut self, pipe: u8, spi: &mut SPI) -> bool {
        if (self.read_byte_register(Register::FifoStatus, spi) & 1) > 0 {
            return false;
        }

        self.read_byte_register(Register::get_payload_length(pipe), spi) == 0
    }

    pub fn power_up(&mut self, spi: &mut SPI, delay: &mut DELAY) {
        let mut config = self.get_config(spi);
        if config & 0b0000_0010 == 0 {
            // Power up and wait for power up
            config |= 0b0000_0010;
            self.write_byte_register(config, Register::Config, spi);

            delay.delay_us(POWERUP_DELAY_US);
        }
    }

    pub fn power_down(&mut self, spi: &mut SPI) {
        let _ = self.ce.set_low();
        let mut config = self.read_byte_register(Register::Config, spi);
        config |= !(1 << 1);
        self.write_byte_register(config, Register::Config, spi);
    }

    pub fn start_listening(&mut self, spi: &mut SPI, delay: &mut DELAY) {
        // Make sure the chip is turned on
        self.power_up(spi, delay);

        let mut config = self.get_config(spi);
        config |= 0b0000_0001;
        self.write_byte_register(config, Register::Config, spi);
        self.write_byte_register(0b0111_0000, Register::Status, spi);
        let _ = self.ce.set_high();
    }

    pub fn stop_listening(&mut self, spi: &mut SPI, delay: &mut DELAY) {
        let _ = self.ce.set_low();
        delay.delay_us(100);

        self.flush_tx(spi);
        let mut config = self.get_config(spi);
        config &= !(1);
        self.write_byte_register(config, Register::Config, spi);
    }

    pub fn flush_rx(&mut self, spi: &mut SPI) -> u8 {
        self.write_command(0xFF, Command::FlushRx, spi)
    }

    pub fn flush_tx(&mut self, spi: &mut SPI) -> u8 {
        self.write_command(0xFF, Command::FlushTx, spi)
    }

    pub fn get_status(&mut self, spi: &mut SPI) -> u8 {
        self.write_command(0xFF, Command::Nop, spi)
    }

    pub fn get_config(&mut self, spi: &mut SPI) -> u8 {
        self.read_byte_register(Register::Config, spi)
    }

    fn read_register(&mut self, register: Register, buffer: &mut [u8], spi: &mut SPI) -> Result<(), RadioError> {
        match buffer.len() {
            1 => {
                let mut read_buffer = [Command::ReadRegister(register as u8).opcode(), 0x00];
                self.safe_transfer_spi(&mut read_buffer, spi);
                buffer[0] = read_buffer[1];
                Ok(())
            },
            5 => {
                let mut read_buffer = [0u8; 6];
                read_buffer[0] = Command::ReadRegister(register as u8).opcode();
                self.safe_transfer_spi(&mut read_buffer, spi);
                buffer[..].copy_from_slice(&read_buffer[1..]);
                Ok(())
            },
            _ => Err(RadioError::InvalidBufferLength),
        }
    }

    fn read_byte_register(&mut self, register: Register, spi: &mut SPI) -> u8 {
        let mut read_buffer = [Command::ReadRegister(register as u8).opcode(), 0x00];
        self.safe_transfer_spi(&mut read_buffer, spi);
        read_buffer[0]
    }

    fn write_command(&mut self, value: u8, command: Command, spi: &mut SPI) -> u8 {
        let mut write_buffer = [command.opcode(), value];
        self.safe_write_spi(&write_buffer, spi);
        write_buffer[0]
    }

    fn write_byte_register(&mut self, value: u8, register: Register, spi: &mut SPI) {
        let write_buffer = [Command::WriteRegister(register as u8).opcode(), value];
        self.safe_write_spi(&write_buffer, spi);
    }

    fn write_register(&mut self, buffer: &[u8], register: Register, spi: &mut SPI) -> Result<(), RadioError> {
        match buffer.len() {
            1 => {
                let write_buffer = [Command::WriteRegister(register as u8).opcode(), buffer[0]];
                self.safe_write_spi(&write_buffer, spi);
                Ok(())
            },
            5 => {
                let mut write_buffer = [0u8; 6];
                write_buffer[0] = Command::WriteRegister(register as u8).opcode();
                write_buffer[1..].copy_from_slice(&buffer[..]);
                self.safe_write_spi(&write_buffer, spi);
                Ok(())
            },
            _ => Err(RadioError::InvalidBufferLength),
        }
    }

    fn safe_write_spi(&mut self, data: &[u8], spi: &mut SPI) {
        let _ = self.csn.set_low();
        let mut result = spi.write(data);
        while let Err(_) = result {
            result = spi.write(data);
        }
        let _ = self.csn.set_high();
    }

    fn safe_transfer_spi(&mut self, buffer: &mut [u8], spi: &mut SPI) {
        let _ = self.csn.set_low();
        let mut result = spi.transfer(buffer);
        while let Err(_) = result {
            result = spi.transfer(buffer);
        }
        let _ = self.csn.set_high();
    }
}