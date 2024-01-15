//!
//! Implementation of the nRF24L01+ Module utilizing rtic monotonics
//! non-blocking delay whenever possible
//! 

#![no_std]

#[cfg(all(feature = "systick", feature = "rp2040"))]
compile_error!("systick and rp2040 are two ways of doing the same thing and are, therefore, mutually exclusive.  Please choose one or the other");

use core::marker::PhantomData;

use config::RegisterValue;
use config::bit_correction::BitCorrection;
use config::data_pipe::DataPipeConfig;
use config::data_rate::DataRate;
use config::interrupt_mask::InterruptMask;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::blocking::delay::{DelayUs, DelayMs};

pub mod command;
use command::Command;

pub mod state;
use state::State;

pub mod config;
use config::{Configuration, RegisterMask};

pub mod error;
use error::Error;

pub mod register;
use register::{ContainsStatus, ConfigRegister, FifoStatusRegister};

#[cfg(feature = "systick")]
use rtic_monotonics::systick::*;
#[cfg(feature = "systick")]
use rtic_monotonics::systick::ExtU32;

#[cfg(feature = "rp2040")]
use rtic_monotonics::rp2040::*;
#[cfg(feature = "rp2040")]
use rtic_monotonics::rp2040::ExtU64;

/// NRF24L01 Driver Implementation
pub struct NRF24L01<
    'a,
    GPIOE,
    SPIE,
    CSN: OutputPin<Error = GPIOE>,
    CE: OutputPin<Error = GPIOE>,
    SPI: Transfer<u8, Error = SPIE> + Write<u8, Error = SPIE>,
    DELAY: DelayUs<u32> + DelayMs<u32>> {
    csn: Option<CSN>,
    ce: CE,
    ce_high: bool,
    state: State,
    configuration: Configuration<'a>,
    phantom: PhantomData<(SPI, DELAY)>,
}

impl<'a, GPIOE, SPIE, CSN, CE, SPI, DELAY> NRF24L01<'a, GPIOE, SPIE, CSN, CE, SPI, DELAY> where
    CSN: OutputPin<Error = GPIOE>,
    CE: OutputPin<Error = GPIOE>,
    SPI: Transfer<u8, Error = SPIE> + Write<u8, Error = SPIE>,
    DELAY: DelayUs<u32> + DelayMs<u32> {
    pub fn new(csn: Option<CSN>, mut ce: CE, config: Configuration<'a>, spi: &mut SPI, delay: &mut DELAY) -> Result<Self, Error<GPIOE, SPIE>> {
        // Wait for power on reset
        delay.delay_us(200_000u32);

        ce.set_low().map_err(Error::GpioError)?;

        let mut driver = Self {
            csn,
            ce,
            ce_high: false,
            state: State::PowerDown,
            configuration: config,
            phantom: PhantomData,
        };

        let mut config = driver.read_config(spi)?;

        if !config.contains_status(ConfigRegister::PowerUp) {
            config |= ConfigRegister::PowerUp as u8;
            driver.write_register(0x00, &[config], spi)?;
            delay.delay_us(2_000u32);
        }

        driver.state = State::Standby1;

        driver.write_full_config(spi, delay)?;

        Ok(driver)
    }

    /// Enqueue a Packet into the Radio FIFO
    pub fn send_packet(&mut self, packet: &[u8], auto_ack: bool, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        // Ensure value payload length
        if packet.len() > 32 {
            return Err(Error::InvalidDataSize);
        }

        // Format the Packet
        let mut send_buffer = [0u8; 33];
        if auto_ack {
            send_buffer[0] = Command::WriteTxPayload.opcode();
        } else {
            send_buffer[0] = Command::WriteTxNoAck.opcode();
        }
        send_buffer[1..=packet.len()].copy_from_slice(packet);

        // Write the Payload to the FIFO
        self.safe_transfer_spi(spi, &mut send_buffer[..=packet.len()])?;

        Ok(send_buffer[0])
    }

    /// Read a packet from the Rx FIFO returning (data.len(), rx_pipe)
    pub fn receive_packet(&mut self, buffer: &mut [u8], spi: &mut SPI) -> Result<(u8, u8), Error<GPIOE, SPIE>> {
        // Make Sure buffer is right size
        if buffer.len() != 32 {
            return Err(Error::InvalidBufferSize);
        }

        // Make Sure in Rx State
        if self.state != State::Rx {
            return Err(Error::SwitchToRx);
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

        buffer[..].copy_from_slice(&rx_buffer[1..]);

        Ok((data_length, pipe_num))
    }

    pub fn set_interrupt_mask(&mut self, interrupt_mask: InterruptMask, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        self.configuration.interrupt_mask = interrupt_mask;

        let mut register_value = [0u8];
        self.read_register(0x00, &mut register_value, spi)?;
        register_value[0] &= 0b1000_1111;
        register_value[0] |= interrupt_mask.register_value(0x00);
        self.write_register(0x00, &register_value, spi)
    }

    pub fn set_bit_correction(&mut self, bit_correction: Option<BitCorrection>, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        self.configuration.bit_correction = bit_correction;

        let mut register_value = [0u8];
        self.read_register(0x00, &mut register_value, spi)?;
        register_value[0] &= 0b1111_0011;
        if let Some(bit_correction) = bit_correction {
            register_value[0] |= bit_correction.register_value(0x00);
        }

        self.write_register(0x00, &register_value, spi)
    }

    pub fn set_pipe_config(&mut self, pipe: u8, pipe_config: Option<DataPipeConfig<'a>>, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        if pipe > 6 {
            return Err(Error::InvalidPipeId);
        }

        if let Some(pipe_config) = pipe_config {
            if pipe == 0 {
                if pipe_config.address.len() != self.configuration.address_width as usize {
                    return Err(Error::InvalidAddressBufferSize);
                }
            } else {
                if pipe_config.address.len() != 1 {
                    return Err(Error::InvalidAddressBufferSize);
                }
            }

            // Set auto acknowledge
            let mut auto_ack = [0u8];
            self.read_register(0x01, &mut auto_ack, spi)?;
            auto_ack[0] &= 0b1111_1111 ^ (1 << pipe);
            auto_ack[0] |= (pipe_config.auto_acknowledge as u8) << pipe;
            self.write_register(0x01, &auto_ack, spi)?;

            // Set Enable
            let mut enabled = [0u8];
            self.read_register(0x02, &mut enabled, spi)?;
            enabled[0] &= 0b1111_1111 ^ (1 << pipe);
            enabled[0] |= (pipe_config.enabled as u8) << pipe;
            self.write_register(0x02, &enabled, spi)?;

            // Set Dynamic Payload
            let mut dpl = [0u8];
            self.read_register(0x1C, &mut dpl, spi)?;
            dpl[0] &= 0b1111_1111 ^ (1 << pipe);
            dpl[0] |= (pipe_config.dynamic_payload as u8) << pipe;
            self.write_register(0x1C, &dpl, spi)?;

            self.configuration.pipe_configs[pipe as usize] = Some(pipe_config);
            
            self.write_register(0x0A + pipe, pipe_config.address, spi)
        } else {
            // Set Auto Acknowledge
            let mut auto_ack = [0u8];
            self.read_register(0x01, &mut auto_ack, spi)?;
            auto_ack[0] &= 0b1111_1111 ^ (1 << pipe);
            self.write_register(0x01, &auto_ack, spi)?;

            // Set Enable
            let mut enabled = [0u8];
            self.read_register(0x02, &mut enabled, spi)?;
            enabled[0] &= 0b1111_1111 ^ (1 << pipe);
            self.write_register(0x02, &enabled, spi)?;

            // Set Dynamic Payload
            let mut dpl = [0u8];
            self.read_register(0x1C, &mut dpl, spi)?;
            dpl[0] &= 0b1111_1111 ^ (1 << pipe);
            self.write_register(0x1C, &dpl, spi)?;

            self.configuration.pipe_configs[pipe as usize] = pipe_config;

            // Set Address
            if pipe == 0 {
                self.write_register(0x0A, &[0u8; 5], spi)
            } else {
                self.write_register(0x0A + pipe, &[0u8], spi)
            }
        }
    }

    pub fn set_data_rate(&mut self, data_rate: DataRate, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        self.configuration.data_rate = data_rate;

        let mut register_value = [0u8];
        self.read_register(0x06, &mut register_value, spi)?;
        register_value[0] &= 0b1101_0111;
        register_value[0] |= data_rate.register_value(0x06);
        self.write_register(0x06, &register_value, spi)
    }

    pub fn set_rf_channel(&mut self, channel: u8, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        if channel >= 128 {
            return Err(Error::InvalidRfChannel);
        }

        self.configuration.rf_channel = channel;
        
        let mut register_value = [0u8];
        self.read_register(0x05, &mut register_value, spi)?;
        register_value[0] &= 0b1000_0000;
        register_value[0] |= channel;
        self.write_register(0x05, &register_value, spi)
    }

    pub fn set_tx_address(&mut self, address: &'a [u8], spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        self.configuration.tx_address = address;

        self.write_register(0x10, address, spi)
    }

    pub fn set_retransmit_delay(&mut self, delay: u8, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        if delay > 31 {
            return Err(Error::InvalidRetransmitDelay);
        }

        self.configuration.retransmit_delay = delay;

        let mut register_value = [0u8];
        self.read_register(0x04, &mut register_value, spi)?;
        register_value[0] &= 0b0000_1111;
        register_value[0] |= delay << 4;
        self.write_register(0x04, &register_value, spi)
    }

    pub fn set_retransmit_count(&mut self, count: u8, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        if count > 31 {
            return Err(Error::InvalidRetransmitCount);
        }

        self.configuration.retransmit_count = count;

        let mut register_value = [0u8];
        self.read_register(0x04, &mut register_value, spi)?;
        register_value[0] &= 0b1111_0000;
        register_value[0] |= count;
        self.write_register(0x04, &register_value, spi)
    }

    pub fn set_dynamic_payload(&mut self, value: bool, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        self.configuration.dynamic_payload = value;

        let mut register_value = [0u8];
        self.read_register(0x1D, &mut register_value, spi)?;
        register_value[0] &= 0b1111_1011;
        register_value[0] |= (value as u8) << 3;
        self.write_register(0x1D, &register_value, spi)
    }

    pub fn set_auto_ack(&mut self, value: bool, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        self.configuration.acknowledge_payload = value;

        let mut register_value = [0u8];
        self.read_register(0x1D, &mut register_value, spi)?;
        register_value[0] &= 0b1111_1101;
        register_value[0] |= (value as u8) << 1;
        self.write_register(0x1D, &register_value, spi)
    }

    pub fn set_dynamic_ack(&mut self, value: bool, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        self.configuration.dynamic_payload = value;

        let mut register_value = [0u8];
        self.read_register(0x1D, &mut register_value, spi)?;
        register_value[0] &= 0b1111_1110;
        register_value[0] |= value as u8;
        self.write_register(0x1D, &register_value, spi)
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
        // Handle Invalid Cases
        if pipe > 5 {
            return Err(Error::InvalidPipeId);
        } else if ack_payload.len() > 32 {
            return Err(Error::TooLargeAckPayload);
        }

        // Write data
        let mut write_data: [u8; 33] = [0u8; 33];
        write_data[0] = Command::WriteAcknowledgePayload(pipe).opcode();
        write_data[1..=ack_payload.len()].copy_from_slice(ack_payload);
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

    pub fn clear_interrupts(&mut self, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
        self.write_register(0x07, &[0b0111_0000], spi)?;
        self.read_status(spi)
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

                buffer[..].copy_from_slice(&status[1..]);
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
                status[1..=data.len()].copy_from_slice(data);
                self.safe_transfer_spi(spi, &mut status)?;

                Ok(status[0])
            },
            _ => return Err(Error::UnknownRegister),
        }
    }

    pub fn to_rx(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), Error<GPIOE, SPIE>> {
        if self.state != State::Rx {
            // Finish sending if there is a send
            if self.state == State::Tx {
                self.wait_for_end_tx(spi, delay)?;
            }

            self.to_state(State::Rx, spi, delay)
        } else {
            Ok(())
        }
    }

    #[cfg(any(feature = "systick", feature = "rp2040"))]
    pub async fn async_to_rx(&mut self, spi: &mut SPI) -> Result<(), Error<GPIOE, SPIE>> {
        if self.state != State::Rx {
            // Finish sending if in tx
            if self.state == State::Tx {
                self.async_wait_for_end_tx(spi).await?;
            }

            self.async_to_state(State::Rx, spi).await
        } else {
            Ok(())
        }
    }

    pub fn to_tx(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), Error<GPIOE, SPIE>> {
        if self.state != State::Tx {
            self.to_state(State::Tx, spi, delay)
        } else {
            Ok(())
        }
    }

    #[cfg(any(feature = "systick", feature = "rp2040"))]
    pub async fn async_to_tx(&mut self, spi: &mut SPI) -> Result<(), Error<GPIOE, SPIE>> {
        if self.state != State::Tx {
            self.async_to_state(State::Tx, spi).await
        } else {
            Ok(())
        }
    }

    pub fn pulse_tx(&mut self, delay: &mut DELAY) -> Result<(), Error<GPIOE, SPIE>> {
        self.ce.set_high().map_err(Error::GpioError)?;
        self.ce_high = true;
        delay.delay_us(11);
        self.ce_high = false;
        self.ce.set_low().map_err(Error::GpioError)
    }

    #[cfg(any(feature = "systick", feature = "rp2040"))]
    pub async fn async_pulse_tx(&mut self) -> Result<(), Error<GPIOE, SPIE>> {
        self.ce_high = true;
        self.ce.set_high().map_err(Error::GpioError)?;
        self.wait_for_us(11).await;
        self.ce_high = true;
        self.ce.set_low().map_err(Error::GpioError)
    }

    // Convert the radio into standby mode
    pub fn to_standby(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), Error<GPIOE, SPIE>> {
        if self.state != State::Standby1 {
            // Finish sending if send in progress
            if self.state == State::Tx {
                self.wait_for_end_tx(spi, delay)?;
            }

            self.to_state(State::Standby1, spi, delay)
        } else {
            Ok(())
        }
    }

    #[cfg(any(feature = "systick", feature = "rp2040"))]
    pub async fn async_to_standby(&mut self, spi: &mut SPI) -> Result<(), Error<GPIOE, SPIE>> {
        if self.state != State::Standby1 {
            // Finish sending if there is a send
            if self.state == State::Tx {
                self.async_wait_for_end_tx(spi).await?;
            }

            self.async_to_state(State::Standby1, spi).await
        } else {
            Ok(())
        }
    }

    // Synchronously Power Down the Radio
    pub fn power_down(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), Error<GPIOE, SPIE>> {
        if self.state != State::PowerDown {
            // Finish sending if send in progress
            if self.state == State::Tx {
                self.wait_for_end_tx(spi, delay)?;
            }

            self.to_state(State::PowerDown, spi, delay)
        } else {
            Ok(())
        }
    }

    #[cfg(any(feature = "systick", feature = "rp2040"))]
    // Asynchronously Power Down the Radio
    pub async fn async_power_down(&mut self, spi: &mut SPI) -> Result<(), Error<GPIOE, SPIE>> {
        if self.state != State::PowerDown {
            // Finish sending if send in progress
            if self.state == State::Tx {
                self.async_wait_for_end_tx(spi).await?;
            }

            self.async_to_state(State::PowerDown, spi).await
        } else {
            Ok(())
        }
    }

    pub fn get_state(&mut self, spi: &mut SPI) -> Result<State, Error<GPIOE, SPIE>> {
        // Read some register
        let config = self.read_config(spi)?;
        let fifo_status = self.read_fifo_status(spi)?;

        match config & 0b0000_0011 {
            3 => {
                if self.ce_high {
                    Ok(State::Rx)
                } else {
                    Ok(State::Standby1)
                }
            },
            2 => {
                if self.ce_high && fifo_status & 0b0001_0000 == 0 {
                    Ok(State::Tx)
                } else if self.ce_high && fifo_status & 0b0001_0000 != 0 {
                    Ok(State::Standby2)
                } else {
                    Ok(State::Standby1)
                }
            }
            _ => Ok(State::PowerDown),
        }
    }

    // Synchronously Wait for Tx FIFO to be Empty
    fn wait_for_end_tx(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), Error<GPIOE, SPIE>> {
        let mut fifo_status = self.read_fifo_status(spi)?;
        while !fifo_status.contains_status(FifoStatusRegister::TxEmpty) {
            // TODO: Optimize this wait time
            delay.delay_ms(5);

            fifo_status = self.read_fifo_status(spi)?;
        }

        Ok(())
    }

    // Asynchronous Wait for Tx FIFO to be Empty
    #[cfg(any(feature = "systick", feature = "rp2040"))]
    async fn async_wait_for_end_tx(&mut self, spi: &mut SPI) -> Result<(), Error<GPIOE, SPIE>> {
        let mut fifo_status = self.read_fifo_status(spi)?;
        while !fifo_status.contains_status(FifoStatusRegister::TxEmpty) {
            // TODO: Optimize this wait time
            self.wait_for_ms(5).await;

            fifo_status = self.read_fifo_status(spi)?;
        }

        Ok(())
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

    fn to_state(&mut self, state: State, spi: &mut SPI, delay: &mut DELAY) -> Result<(), Error<GPIOE, SPIE>> {
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
                self.ce_high = true;
                self.write_register(0x00, &[config], spi)?;
                delay.delay_us(130u32);

                self.state = State::Rx;
            },
            (State::Standby1, State::Tx) => {
                config &= !(ConfigRegister::RxTxControl as u8);
                self.ce.set_high().map_err(Error::GpioError)?;
                self.ce_high = true;
                self.write_register(0x00, &[config], spi)?;
                delay.delay_us(140u32);

                self.state = State::Tx;
            },
            (State::Tx, State::Standby1) | (State::Rx, State::Standby1) => {
                self.ce.set_low().map_err(Error::GpioError)?;
                self.ce_high = false;

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
                self.ce_high = true;
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
                self.ce_high = true;
                self.write_register(0x00, &[config], spi)?;
                delay.delay_us(140u32);

                self.state = State::Tx;
            },
            (State::Rx, State::Tx) => {
                self.ce.set_low().map_err(Error::GpioError)?;
                self.ce_high = false;

                self.state = State::Standby1;

                config &= !(ConfigRegister::RxTxControl as u8);
                self.ce.set_high().map_err(Error::GpioError)?;
                self.ce_high = true;
                self.write_register(0x00, &[config], spi)?;
                delay.delay_us(140u32);

                self.state = State::Tx;
            },
            (State::Tx, State::Rx) => {
                self.ce.set_low().map_err(Error::GpioError)?;
                self.ce_high = false;

                self.state = State::Standby1;

                config |= ConfigRegister::RxTxControl as u8;
                self.ce.set_high().map_err(Error::GpioError)?;
                self.ce_high = true;
                self.write_register(0x00, &[config], spi)?;
                delay.delay_us(130u32);

                self.state = State::Rx;
            },
            _ => (),
        }

        Ok(())
    }

    #[cfg(any(feature = "systick", feaure = "rp2040"))]
    pub async fn async_to_state(&mut self, state: State, spi: &mut SPI) -> Result<(), Error<GPIOE, SPIE>> {
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
                    self.wait_for_us(1500).await;
                }

                self.state = State::Standby1;
            },
            (State::Standby1, State::Rx) => {
                config |= ConfigRegister::RxTxControl as u8;
                self.ce.set_high().map_err(Error::GpioError)?;
                self.ce_high = true;
                self.write_register(0x00, &[config], spi)?;
                self.wait_for_us(130).await;

                self.state = State::Rx;
            },
            (State::Standby1, State::Tx) => {
                config &= !(ConfigRegister::RxTxControl as u8);
                self.ce.set_high().map_err(Error::GpioError)?;
                self.ce_high = true;
                self.write_register(0x00, &[config], spi)?;
                self.wait_for_us(140).await;

                self.state = State::Tx;
            },
            (State::Tx, State::Standby1) | (State::Rx, State::Standby1) => {
                self.ce.set_low().map_err(Error::GpioError)?;
                self.ce_high = false;

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
                    self.wait_for_us(1500).await;
                }

                self.state = State::Standby1;

                config |= ConfigRegister::RxTxControl as u8;
                self.ce.set_high().map_err(Error::GpioError)?;
                self.ce_high = true;
                self.write_register(0x00, &[config], spi)?;
                self.wait_for_us(130).await;

                self.state = State::Rx;
            },
            (State::PowerDown, State::Tx) => {
                if !config.contains_status(ConfigRegister::PowerUp) {
                    config |= ConfigRegister::PowerUp as u8;
                    self.write_register(0x00, &[config], spi)?;
                    self.wait_for_us(1500).await;
                }

                self.state = State::Standby1;

                config &= !(ConfigRegister::RxTxControl as u8);
                self.ce.set_high().map_err(Error::GpioError)?;
                self.ce_high = true;
                self.write_register(0x00, &[config], spi)?;
                self.wait_for_us(140).await;

                self.state = State::Tx;
            },
            (State::Rx, State::Tx) => {
                self.ce.set_low().map_err(Error::GpioError)?;
                self.ce_high = false;

                self.state = State::Standby1;

                config &= !(ConfigRegister::RxTxControl as u8);
                self.ce.set_high().map_err(Error::GpioError)?;
                self.ce_high = true;
                self.write_register(0x00, &[config], spi)?;
                self.wait_for_us(140).await;

                self.state = State::Tx;
            },
            (State::Tx, State::Rx) => {
                self.ce.set_low().map_err(Error::GpioError)?;
                self.ce_high = false;

                self.state = State::Standby1;

                config |= ConfigRegister::RxTxControl as u8;
                self.ce.set_high().map_err(Error::GpioError)?;
                self.ce_high = true;
                self.write_register(0x00, &[config], spi)?;
                self.wait_for_us(130).await;

                self.state = State::Rx;
            },
            _ => (),
        }

        Ok(())
    }

    fn write_full_config(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), Error<GPIOE, SPIE>> {
        // Write first 7 configs
        for register in 0x00..=0x06 {
            let mut config = [0u8];

            self.read_register(register, &mut config, spi)?;
            config[0] &= self.configuration.register_mask(register);
            config[0] |= self.configuration.register_value(register);
            self.write_register(register, &config, spi)?;

            delay.delay_ms(10);

            let mut found_config = [0u8];
            self.read_register(register, &mut found_config, spi)?;

            if found_config[0] != config[0] {
                return Err(Error::UnableToConfigureRegister(register, config[0], found_config[0]));
            }
        }

        // Write Base Register and Feature Registers
        let base_config = self.configuration.pipe_configs[0].unwrap();
        self.write_register(0x0A, base_config.address, spi)?;

        delay.delay_ms(10);

        let mut found_config = [0u8; 5];
        self.read_register(0x0A, &mut found_config, spi)?;

        for i in 0..base_config.address.len() {
            if base_config.address[i] != found_config[i] {
                return Err(Error::UnableToConfigureRegister(0x0A, base_config.address[i], found_config[i]));
            }
        }

        // Write Other Pipe Registers
        let mut pipe_index = 1;
        for register in 0x0B..=0x0F {
            if let Some(pipe_config) = self.configuration.pipe_configs[pipe_index] {
                self.write_register(register, &[pipe_config.address[0]], spi)?;

                delay.delay_ms(10);

                let mut found_config = [0u8];
                self.read_register(register, &mut found_config, spi)?;

                if found_config[0] != pipe_config.address[0] {
                    return Err(Error::UnableToConfigureRegister(register, pipe_config.address[0], found_config[0]));
                }
            } else {
                break;
            }

            pipe_index += 1;
        }

        // Write Transmit Register
        self.write_register(0x10, self.configuration.tx_address, spi)?;

        delay.delay_ms(10);

        let mut found_config = [0u8; 5];
        self.read_register(0x10, &mut found_config, spi)?;

        for i in 0..self.configuration.tx_address.len() {
            if self.configuration.tx_address[i] != found_config[i] {
                return Err(Error::UnableToConfigureRegister(0x10, self.configuration.tx_address[i], found_config[i]));
            }
        }

        // Write Address Registers
        for register in (0x1C..=0x1D).rev() {
            let mut activate_instruction = [Command::Activate.opcode(), 0x73];
            self.safe_transfer_spi(spi, &mut activate_instruction)?;

            let mut config = [0u8];

            self.read_register(register, &mut config, spi)?;
            config[0] &= self.configuration.register_mask(register);
            config[0] |= self.configuration.register_value(register);
            self.write_register(register, &config, spi)?;

            delay.delay_ms(10);

            let mut found_config = [0u8];
            self.read_register(register, &mut found_config, spi)?;

            if found_config[0] != config[0] {
                return Err(Error::UnableToConfigureRegister(register, config[0], found_config[0]));
            }
        }

        Ok(())
    }

    #[cfg(feature = "systick")]
    async fn wait_for_ms(&self, ms: u32) {
        Systick::delay(ms.millis()).await;
    }

    #[cfg(feature = "systick")]
    async fn wait_for_us(&self, us: u32) {
        Systick::delay(us.micros()).await;
    }

    #[cfg(feature = "rp2040")]
    async fn wait_for_ms(&self, ms: u64) {
        Timer::delay(ms.millis()).await;
    }

    #[cfg(feature = "rp2040")]
    async fn wait_for_us(&self, us: u64) {
        Timer::delay(us.micros()).await;
    }
}

// /// NRF24L01 Driver Implementation With Rtic Monotnic Delays
// #[cfg(any(feature = "systick", feature = "rp2040"))]
// pub struct NRF24L01<
//     'a,
//     GPIOE,
//     SPIE,
//     CSN: OutputPin<Error = GPIOE>,
//     CE: OutputPin<Error = GPIOE>,
//     SPI: Transfer<u8, Error = SPIE> + Write<u8, Error = SPIE>> {
//     csn: Option<CSN>,
//     ce: CE,
//     state: State,
//     configuration: Configuration<'a>,
//     phantom: PhantomData<SPI>,
// }

// #[cfg(any(feature = "systick", feature = "rp2040"))]
// impl<'a, GPIOE, SPIE, CSN, CE, SPI> NRF24L01<'a, GPIOE, SPIE, CSN, CE, SPI> where
//     CSN: OutputPin<Error = GPIOE>,
//     CE: OutputPin<Error = GPIOE>,
//     SPI: Transfer<u8, Error = SPIE> + Write<u8, Error = SPIE> {
//     pub async fn new(csn: Option<CSN>, ce: CE, config: Configuration<'a>, spi: &mut SPI) -> Result<Self, (u8, Error<GPIOE, SPIE>)> {
//         // Wait for power on reset
//         #[cfg(feature = "systick")]
//         Systick::delay(200u32.millis()).await;
//         #[cfg(feature = "rp2040")]
//         Timer::delay(200u64.millis()).await;

//         let mut driver = Self {
//             csn,
//             ce,
//             state: State::PowerDown,
//             configuration: config,
//             phantom: PhantomData,
//         };

//         let mut config = driver.read_config(spi).await.map_err(|err| { (0u8, err) })?;

//         if !config.contains_status(ConfigRegister::PowerUp) {
//             config |= ConfigRegister::PowerUp as u8;
//             driver.write_register(0x00, &[config], spi).await.map_err(|err| { (1u8, err) })?;

//             #[cfg(feature = "systick")]
//             Systick::delay(2000u32.micros()).await;
//             #[cfg(feature = "rp2040")]
//             Timer::delay(2000u64.micros()).await;
//         }

//         driver.state = State::Standby1;

//         driver.write_full_config(spi).await?;

//         Ok(driver)
//     }

//     /// Set the interrupt mask of the nRF24L01 and return the status
//     pub async fn set_interrupt_mask(&mut self, interrupt_mask: InterruptMask, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
//         self.configuration.interrupt_mask = interrupt_mask;

//         let mut register_value = [0u8];
//         self.read_register(0x00, &mut register_value, spi).await?;
//         register_value[0] &= 0b1000_1111;
//         register_value[0] |= interrupt_mask.register_value(0x00);
//         self.write_register(0x00, &register_value, spi).await
//     }
    
//     /// Set the bit correction level and method and return the status
//     pub async fn set_bit_correction(&mut self, bit_correction: Option<BitCorrection>, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
//         self.configuration.bit_correction = bit_correction;

//         let mut register_value = [0u8];
//         self.read_register(0x00, &mut register_value, spi).await?;
//         register_value[0] &= 0b1111_0011;
//         if let Some(bit_correction) = bit_correction {
//             register_value[0] |= bit_correction.register_value(0x00);
//         }

//         self.write_register(0x00, &register_value, spi).await
//     }

//     /// Set the pipe configuration of a specific pipe and return the status
//     pub async fn set_pipe_config(&mut self, pipe: u8, pipe_config: Option<DataPipeConfig<'a>>, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
//         if pipe > 6 {
//             return Err(Error::InvalidPipeId);
//         }

//         if let Some(pipe_config) = pipe_config {
//             if pipe == 0 {
//                 if pipe_config.address.len() != self.configuration.address_width.as_u8() as usize {
//                     return Err(Error::InvalidAddressBufferSize);
//                 }
//             } else {
//                 if pipe_config.address.len() != 1 {
//                     return Err(Error::InvalidAddressBufferSize);
//                 }
//             }

//             // Set auto acknowledge
//             let mut auto_ack = [0u8];
//             self.read_register(0x01, &mut auto_ack, spi).await?;
//             auto_ack[0] &= 0b1111_1111 ^ (1 << pipe);
//             auto_ack[0] |= (pipe_config.auto_acknowledge as u8) << pipe;
//             self.write_register(0x01, &auto_ack, spi).await?;

//             // Set Enable
//             let mut enabled = [0u8];
//             self.read_register(0x02, &mut enabled, spi).await?;
//             enabled[0] &= 0b1111_1111 ^ (1 << pipe);
//             enabled[0] |= (pipe_config.enabled as u8) << pipe;
//             self.write_register(0x02, &enabled, spi).await?;

//             // Set Dynamic Payload
//             let mut dpl = [0u8];
//             self.read_register(0x1C, &mut dpl, spi).await?;
//             dpl[0] &= 0b1111_1111 ^ (1 << pipe);
//             dpl[0] |= (pipe_config.dynamic_payload as u8) << pipe;
//             self.write_register(0x1C, &dpl, spi).await?;

//             self.configuration.pipe_configs[pipe as usize] = Some(pipe_config);

//             self.write_register(0x0A + pipe, pipe_config.address, spi).await
//         } else {
//             // Set Auto Acknowledge
//             let mut auto_ack = [0u8];
//             self.read_register(0x01, &mut auto_ack, spi).await?;
//             auto_ack[0] &= 0b1111_1111 ^ (1 << pipe);
//             self.write_register(0x01, &auto_ack, spi).await?;

//             // Set Enable
//             let mut enabled = [0u8];
//             self.read_register(0x02, &mut enabled, spi).await?;
//             enabled[0] &= 0b1111_1111 ^ (1 << pipe);
//             self.write_register(0x02, &enabled, spi).await?;

//             // Set Dynamic Payload
//             let mut dpl = [0u8];
//             self.read_register(0x1C, &mut dpl, spi).await?;
//             dpl[0] &= 0b1111_1111 ^ (1 << pipe);
//             self.write_register(0x1C, &dpl, spi).await?;

//             self.configuration.pipe_configs[pipe as usize] = pipe_config;

//             // Set Address
//             if pipe == 0 {
//                 self.write_register(0x0A, &[0u8; 5], spi).await
//             } else {
//                 self.write_register(0x0A + pipe, &[0u8], spi).await
//             }
//         }
//     }

//     /// Set the data rate for the nRF24L01 module
//     pub async fn set_data_rate(&mut self, data_rate: DataRate, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
//         self.configuration.data_rate = data_rate;

//         let mut register_value = [0u8];
//         self.read_register(0x06, &mut register_value, spi).await?;
//         register_value[0] &= 0b1101_0111;
//         register_value[0] |= data_rate.register_value(0x06);
//         self.write_register(0x06, &register_value, spi).await
//     }

//     /// Set the channel the nRF24L01 module is operating on
//     pub async fn set_rf_channel(&mut self, channel: u8, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
//         if channel >= 128 {
//             return Err(Error::InvalidRfChannel);
//         }

//         self.configuration.rf_channel = channel;

//         let mut register_value = [0u8];
//         self.read_register(0x05, &mut register_value, spi).await?;
//         register_value[0] &= 0b1000_0000;
//         register_value[0] |= channel;
//         self.write_register(0x05, &register_value, spi).await
//     }

//     /// Set the tx address of the nRF24L01 module
//     pub async fn set_tx_address(&mut self, address: &'a [u8], spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
//         self.configuration.tx_address = address;

//         self.write_register(0x10, address, spi).await
//     }

//     /// Set the retransmit delay of the nRF24L01 module
//     pub async fn set_retransmit_delay(&mut self, delay: u8, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
//         if delay > 31 {
//             return Err(Error::InvalidRetransmitDelay);
//         }

//         self.configuration.retransmit_delay = delay;

//         let mut register_value = [0u8];
//         self.read_register(0x04, &mut register_value, spi).await?;
//         register_value[0] &= 0b0000_1111;
//         register_value[0] |= delay << 4;
//         self.write_register(0x04, &register_value, spi).await
//     }

//     /// Set the retransmit count of the nRF24L01 module
//     pub async fn set_retransmit_count(&mut self, count: u8, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
//         if count > 31 {
//             return Err(Error::InvalidRetransmitCount);
//         }

//         self.configuration.retransmit_count = count;

//         let mut register_value = [0u8];
//         self.read_register(0x04, &mut register_value, spi).await?;
//         register_value[0] &= 0b1111_0000;
//         register_value[0] |= count;
//         self.write_register(0x04, &register_value, spi).await
//     }

//     /// Turn on or off dynamic payload length
//     pub async fn set_dynamic_payload(&mut self, value: bool, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
//         self.configuration.dynamic_payload = value;

//         let mut register_value = [0u8];
//         self.read_register(0x1D, &mut register_value, spi).await?;
//         register_value[0] &= 0b1111_1011;
//         register_value[0] |= (value as u8) << 3;
//         self.write_register(0x1D, &register_value, spi).await
//     }

//     /// Turn on or off auto acknowledge
//     pub async fn set_auto_ack(&mut self, value: bool, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
//         self.configuration.acknowledge_payload = value;

//         let mut register_value = [0u8];
//         self.read_register(0x1D, &mut register_value, spi).await?;
//         register_value[0] &= 0b1111_1101;
//         register_value[0] |= (value as u8) << 1;
//         self.write_register(0x1D, &register_value, spi).await
//     }

//     /// Turn on or off dynamic acknowledgements
//     pub async fn set_dynamic_ack(&mut self, value: bool, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
//         self.configuration.dynamic_payload = value;

//         let mut register_value = [0u8];
//         self.read_register(0x1D, &mut register_value, spi).await?;
//         register_value[0] &= 0b1111_1110;
//         register_value[0] |= value as u8;
//         self.write_register(0x1D, &register_value, spi).await
//     }

//     pub async fn read_register(&mut self, register: u8, buffer: &mut [u8], spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
//         match register {
//             // 1 Byte Registers
//             0x00..=0x09 | 0x0B..=0x0F | 0x11..=0x17 | 0x1C..=0x1D => {
//                 if buffer.len() != 1 {
//                     return Err(Error::InvalidBufferSize);
//                 }

//                 let mut status = [Command::ReadRegister(register).opcode(), 0x00];
//                 self.safe_transfer_spi(spi, &mut status).await?;

//                 buffer[0] = status[1];
//                 Ok(status[0])
//             },
//             // Address Registers (5 Byte Maximum)
//             0x0A | 0x10 => {
//                 if buffer.len() != 5 {
//                     return Err(Error::InvalidBufferSize);
//                 }

//                 let mut status = [0u8; 6];
//                 status[0] = Command::ReadRegister(register).opcode();
//                 self.safe_transfer_spi(spi, &mut status).await?;

//                 buffer[..].copy_from_slice(&status[1..]);
//                 Ok(status[0])
//             },
//             _ => return Err(Error::UnknownRegister),
//         }
//     }

//     pub async fn write_register(&mut self, register: u8, data: &[u8], spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
//         match register {
//             // 1 Byte Registers
//             0x00..=0x09 | 0x0B..=0x0F | 0x1C..=0x1D => {
//                 if data.len() != 1 {
//                     return Err(Error::InvalidRegisterBufferSize(register));
//                 }

//                 let mut status = [Command::WriteRegister(register).opcode(), data[0]];
//                 self.safe_transfer_spi(spi, &mut status).await?;

//                 Ok(status[0])
//             },
//             // Address Registers (5 Byte Maximum)
//             0x0A | 0x10 => {
//                 if data.len() != self.configuration.address_width.as_u8() as usize {
//                     return Err(Error::InvalidAddressBufferSize);
//                 }

//                 let mut status = [0u8; 6];
//                 status[0] = Command::WriteRegister(register).opcode();
//                 status[1..1+data.len()].copy_from_slice(data);
//                 self.safe_transfer_spi(spi, &mut status).await?;

//                 Ok(status[0])
//             },
//             _ => return Err(Error::UnknownRegister),
//         }
//     }

//     /// Read a piece of data from the Rx FIFO of max length 32 returning (data.len(), rx_pipe)
//     pub async fn read_data(&mut self, data_buffer: &mut [u8], spi: &mut SPI) -> Result<(u8, u8), Error<GPIOE, SPIE>> {
//         // Convert to Rx State
//         self.to_rx(spi).await?;

//         // Make sure the data_buffer is not too large
//         if data_buffer.len() != 32 {
//             return Err(Error::InvalidBufferSize);
//         }

//         // Read the incoming data pipe number
//         let status = self.read_status(spi).await?;
//         let pipe_num = status & 0b0000_1110;

//         // Read the incoming data length
//         let data_length = self.read_payload_length(spi).await?;

//         // Read the rx data
//         let mut rx_buffer = [0u8; 33];
//         rx_buffer[0] = Command::ReadRxPayload.opcode();
//         self.safe_transfer_spi(spi, &mut rx_buffer).await?;

//         // Transfer the data into the buffer
//         data_buffer[..].copy_from_slice(&rx_buffer[1..]);

//         Ok((data_length, pipe_num))
//     }

//     /// Read data from the device from a given pipe until that pipe is no longer the next incoming data.
//     pub async fn read_from_pipe(&mut self, pipe: u8, buffer: &mut [u8], spi: &mut SPI) -> Result<usize, Error<GPIOE, SPIE>> {
//         // Convert to Rx State
//         self.to_rx(spi).await?;

//         // Keep track of where in the buffer we're writing to
//         let mut buffer_idx = 0usize;

//         let status = self.read_status(spi).await?;
//         let mut pipe_num = status & 0b0000_1110;
//         while pipe_num == pipe {
//             // Read the incoming data length
//             let data_length = self.read_payload_length(spi).await?;

//             // Check whether we can actually read the next piece of data
//             if data_length as usize + buffer_idx >= buffer.len() {
//                 return Ok(buffer.len() - 1 - buffer_idx);
//             }

//             // Read the rx data
//             let mut rx_buffer = [0u8; 33];
//             rx_buffer[0] = Command::ReadRxPayload.opcode();
//             self.safe_transfer_spi(spi, &mut rx_buffer).await?;

//             // Transfer the data into the buffer
//             buffer[buffer_idx..(buffer_idx + data_length as usize)].copy_from_slice(&rx_buffer[1..(1+data_length as usize)]);

//             // Increment the buffer_idx
//             buffer_idx += data_length as usize;

//             // Update the next fx pipe number
//             let status = self.read_status(spi).await?;
//             pipe_num = status & 0b0000_1110;
//         }

//         Ok(buffer_idx)
//     }

//     /// Send a payload of up to 32 bytes with or without auto acknowledgement
//     pub async fn send_data(&mut self, payload: &[u8], auto_ack: bool, spi: &mut SPI) -> Result<u8, (u8, Error<GPIOE, SPIE>)> {
//         // Convert to Tx State
//         if self.state != State::Tx {
//             self.wait_for_ms(1).await;
//             self.to_state(State::Tx, spi).await.map_err(|err| { (0u8, err) })?;
//         }

//         // Make sure the data is not too large
//         if payload.len() > 32 {
//             return Err((1u8, Error::InvalidDataSize));
//         }

//         // Wait until the tx FIFO is not full
//         self.wait_for_ms(1).await;
//         let mut fifo_status = self.read_fifo_status(spi).await.map_err(|err| { (2u8, err) })?;
//         while fifo_status.contains_status(FifoStatusRegister::TxFull) {
//             // Wait for the fifo to not be empty
//             // TODO: Optimize this value
//             #[cfg(feature = "systick")]
//             Systick::delay(10u32.millis()).await;
//             #[cfg(feature = "rp2040")]
//             Timer::delay(10u64.millis()).await;

//             fifo_status = self.read_fifo_status(spi).await.map_err(|err| { (3u8, err)})?;
//         }

//         self.wait_for_ms(1).await;

//         // Write the correct command
//         let mut send_buffer = [0u8; 33];
//         if auto_ack {
//             send_buffer[0] = Command::WriteTxPayload.opcode();
//         } else {
//             send_buffer[0] = Command::WriteTxNoAck.opcode();
//         }

//         // Copy in the payload to send
//         send_buffer[1..(1+payload.len())].copy_from_slice(payload);

//         // Send the payload
//         self.safe_transfer_spi(spi, &mut send_buffer[..payload.len()]).await.map_err(|err| { (4u8, err) })?;

//         // Return the device status
//         Ok(send_buffer[0])
//     }

//     // Send a payload of longer than 32 Bytes
//     pub async fn send_payloads(&mut self, payload: &[u8], auto_ack: bool, spi: &mut SPI) -> Result<(), Error<GPIOE, SPIE>> {
//         // Convert to Tx State
//         if self.state != State::Tx {
//             self.to_state(State::Tx, spi).await?;
//         }

//         // Wait until the tx FIFO is not full
//         let mut fifo_status = self.read_fifo_status(spi).await?;
//         while fifo_status.contains_status(FifoStatusRegister::TxFull) {
//             // Wait for the fifo to not be empty
//             // TODO: Optimize this value
//             #[cfg(feature = "systick")]
//             Systick::delay(10u32.millis()).await;
//             #[cfg(feature = "rp2040")]
//             Timer::delay(10u64.millis()).await;

//             fifo_status = self.read_fifo_status(spi).await?;
//         }

//         // Send (payload.len() / 32) 32 Byte payloads and 1 (payload.len() % 32) payload
//         for payload_num in 0..(payload.len() / 32) {
//             let mut send_buffer = [0u8; 33];
//             if auto_ack {
//                 send_buffer[0] = Command::WriteTxPayload.opcode();
//             } else {
//                 send_buffer[0] = Command::WriteTxNoAck.opcode();
//             }

//             // Copy in the payload to send
//             send_buffer[1..].copy_from_slice(&payload[(payload_num * 32)..((payload_num + 1) * 32)]);
//             self.safe_transfer_spi(spi, &mut send_buffer).await?;

//             // Wait until the tx FIFO is not full
//             fifo_status = self.read_fifo_status(spi).await?;
//             while fifo_status.contains_status(FifoStatusRegister::TxFull) {
//                 // Wait for the fifo to not be empty
//                 // TODO: Optimize this value
//                 #[cfg(feature = "systick")]
//                 Systick::delay(10u32.millis()).await;
//                 #[cfg(feature = "rp2040")]
//                 Timer::delay(10u64.millis()).await;

//                 fifo_status = self.read_fifo_status(spi).await?;
//             }
//         }

//         // Send (payload.len() % 32) payload
//         let payload_len = payload.len() % 32;
//         if payload_len == 0 {
//             return Ok(());
//         }

//         let mut send_buffer = [0u8; 33];
//         if auto_ack {
//             send_buffer[0] = Command::WriteTxPayload.opcode();
//         } else {
//             send_buffer[0] = Command::WriteTxNoAck.opcode();
//         }

//         // Copy in the payload to send
//         send_buffer[1..(1+payload_len)].copy_from_slice(&payload[(payload.len()-1-payload_len)..(payload.len()-1)]);
//         self.safe_transfer_spi(spi, &mut send_buffer[0..(1+payload_len)]).await?;

//         Ok(())
//     }

//     pub async fn flush_tx(&mut self, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
//         let mut status = [Command::FlushTx.opcode()];
//         self.safe_transfer_spi(spi, &mut status).await?;
//         Ok(status[0])
//     }

//     pub async fn flush_rx(&mut self, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
//         let mut status = [Command::FlushRx.opcode()];
//         self.safe_transfer_spi(spi, &mut status).await?;
//         Ok(status[0])
//     }

//     pub async fn retransmit_payload(&mut self, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
//         let mut status = [Command::ReuseTxPayload.opcode()];
//         self.safe_transfer_spi(spi, &mut status).await?;
//         Ok(status[0])
//     }

//     pub async fn read_payload_length(&mut self, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
//         let mut data: [u8; 2] = [0u8; 2];
//         data[0] = Command::ReadRxPayloadWidth.opcode();
//         self.safe_transfer_spi(spi, &mut data).await?;
//         Ok(data[1])
//     }

//     pub async fn set_ack_payload(&mut self, pipe: u8, ack_payload: &[u8], spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
//         // Handle invalid cases
//         if pipe > 5 {
//             return Err(Error::InvalidPipeId);
//         } else if ack_payload.len() > 32 {
//             return Err(Error::TooLargeAckPayload);
//         }

//         // Write data in form [opcode, data0, data1, ... data31]
//         let mut write_data: [u8; 33] = [0u8; 33];
//         write_data[0] = Command::WriteAcknowledgePayload(pipe).opcode();
//         write_data[1..=ack_payload.len()].copy_from_slice(ack_payload);

//         self.safe_transfer_spi(spi, &mut write_data).await?;

//         Ok(write_data[0])
//     }

//     pub async fn read_status(&mut self, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
//         let mut status = [Command::Nop.opcode()];
//         self.safe_transfer_spi(spi, &mut status).await?;
//         Ok(status[0])
//     }

//     pub async fn read_config(&mut self, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
//         let mut config = [Command::ReadRegister(0x00).opcode(), 0x00];
//         self.safe_transfer_spi(spi, &mut config).await?;
//         Ok(config[1])
//     }

//     pub async fn read_fifo_status(&mut self, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
//         let mut status = [Command::ReadRegister(0x17).opcode(), 0x00];
//         self.safe_transfer_spi(spi, &mut status).await?;
//         Ok(status[1])
//     }

//     pub async fn clear_interrupt(&mut self, spi: &mut SPI) -> Result<u8, Error<GPIOE, SPIE>> {
//         self.write_register(0x07, &[0b1011_0000], spi).await?;
//         self.read_status(spi).await
//     }

//     // Convert the nRF24L01 into Rx Mode
//     pub async fn to_rx(&mut self, spi: &mut SPI) -> Result<(), Error<GPIOE, SPIE>> {
//         if self.state != State::Rx {
//             // Finish sending if there is a send
//             if self.state == State::Tx {
//                 self.wait_for_end_tx(spi).await?;
//             }

//             self.to_state(State::Rx, spi).await
//         } else {
//             Ok(())
//         }
//     }

//     // Convert the nRF24L01 into Standby Mode
//     pub async fn to_standby(&mut self, spi: &mut SPI) -> Result<(), Error<GPIOE, SPIE>> {
//         if self.state != State::Standby1 {
//             // Finish sending if there is a send
//             if self.state == State::Tx {
//                 self.wait_for_end_tx(spi).await?;
//             }

//             self.to_state(State::Standby1, spi).await
//         } else {
//             Ok(())
//         }
//     }

//     // Convert the nRF24L01 into Power Down Mode
//     pub async fn power_down(&mut self, spi: &mut SPI) -> Result<(), Error<GPIOE, SPIE>> {
//         if self.state != State::PowerDown {
//             // Finish sending if there is a send
//             if self.state == State::Tx {
//                 self.wait_for_end_tx(spi).await?;
//             }

//             self.to_state(State::PowerDown, spi).await
//         } else {
//             Ok(())
//         }
//     }

//     // Asynchronous wait that checks the Tx FIFO Status
//     async fn wait_for_end_tx(&mut self, spi: &mut SPI) -> Result<(), Error<GPIOE, SPIE>> {
//         let mut fifo_status = self.read_fifo_status(spi).await?;
//         while !fifo_status.contains_status(FifoStatusRegister::TxEmpty) {
//             // TODO: Optimize this wait time
//             #[cfg(feature = "systick")]
//             Systick::delay(5u32.millis()).await;
//             #[cfg(feature = "rp2040")]
//             Timer::delay(5u64.millis()).await;

//             fifo_status = self.read_fifo_status(spi).await?;
//         }

//         Ok(())
//     }

//     async fn safe_transfer_spi(&mut self, spi: &mut SPI, buffer: &mut [u8]) -> Result<(), Error<GPIOE, SPIE>> {
//         // For some reason the imxrt hal doesn't block and will return busy when using the spi
//         #[cfg(feature = "systick")]
//         self.wait_for_ms(1u32).await;

//         match self.csn.as_mut() {
//             Some(csn) => {
//                 csn.set_low().map_err(Error::GpioError)?;
//                 let transfer_result = spi.transfer(buffer);
//                 let gpio_error = csn.set_high();

//                 match (transfer_result, gpio_error) {
//                     (Err(err), Ok(_)) => Err(Error::SpiError(err)),
//                     (Ok(_), Err(err)) => Err(Error::GpioError(err)),
//                     (Err(spi_err), Err(gpio_err)) => Err(Error::GpioSpiError((gpio_err, spi_err))),
//                     _ => Ok(()),
//                 }
//             },
//             None => {
//                 spi.transfer(buffer).map_err(Error::SpiError)?;
//                 Ok(())
//             }
//         }
//     }

//     async fn to_state(&mut self, state: State, spi: &mut SPI) -> Result<(), Error<GPIOE, SPIE>> {
//         if state == self.state {
//             return Ok(());
//         }

//         let mut config = self.read_config(spi).await?;

//         // There are a few filled in cases that could be done with recursion, but recursion with async functions is
//         // no bueno
//         match (self.state, state) {
//             (State::PowerDown, State::Standby1) => {
//                 if !config.contains_status(ConfigRegister::PowerUp) {
//                     config |= ConfigRegister::PowerUp as u8;
//                     self.write_register(0x00, &[config], spi).await?;
//                     #[cfg(feature = "systick")]
//                     Systick::delay(1500u32.micros()).await;
//                     #[cfg(feature = "rp2040")]
//                     Timer::delay(1500u64.micros()).await;
//                 }

//                 self.state = State::Standby1;
//             },
//             (State::Standby1, State::Rx) => {
//                 config |= ConfigRegister::RxTxControl as u8;
//                 self.ce.set_high().map_err(Error::GpioError)?;
//                 self.write_register(0x00, &[config], spi).await?;
//                 #[cfg(feature = "systick")]
//                 Systick::delay(130u32.micros()).await;
//                 #[cfg(feature = "rp2040")]
//                 Timer::delay(130u64.micros()).await;

//                 self.state = State::Rx;
//             },
//             (State::Standby1, State::Tx) => {
//                 config &= !(ConfigRegister::RxTxControl as u8);
//                 self.ce.set_high().map_err(Error::GpioError)?;
//                 self.write_register(0x00, &[config], spi).await?;
//                 #[cfg(feature = "systick")]
//                 Systick::delay(140u32.micros()).await;
//                 #[cfg(feature = "rp2040")]
//                 Timer::delay(140u64.micros()).await;

//                 self.state = State::Tx;
//             },
//             (State::Tx, State::Standby1) | (State::Rx, State::Standby1) => {
//                 self.ce.set_low().map_err(Error::GpioError)?;

//                 self.state = State::Standby1;
//             },
//             (State::Standby1, State::PowerDown) => {
//                 config &= !(ConfigRegister::PowerUp as u8);
//                 self.write_register(0x00, &[config], spi).await?;
                
//                 self.state = State::PowerDown;
//             },
//             (State::PowerDown, State::Rx) => {
//                 if !config.contains_status(ConfigRegister::PowerUp) {
//                     config |= ConfigRegister::PowerUp as u8;
//                     self.write_register(0x00, &[config], spi).await?;
//                     #[cfg(feature = "systick")]
//                     Systick::delay(1500u32.micros()).await;
//                     #[cfg(feature = "rp2040")]
//                     Timer::delay(1500u64.micros()).await;
//                 }

//                 self.state = State::Standby1;

//                 config |= ConfigRegister::RxTxControl as u8;
//                 self.ce.set_high().map_err(Error::GpioError)?;
//                 self.write_register(0x00, &[config], spi).await?;
//                 #[cfg(feature = "systick")]
//                 Systick::delay(130u32.micros()).await;
//                 #[cfg(feature = "rp2040")]
//                 Timer::delay(130u64.micros()).await;

//                 self.state = State::Rx;
//             },
//             (State::PowerDown, State::Tx) => {
//                 if !config.contains_status(ConfigRegister::PowerUp) {
//                     config |= ConfigRegister::PowerUp as u8;
//                     self.write_register(0x00, &[config], spi).await?;
//                     #[cfg(feature = "systick")]
//                     Systick::delay(1500u32.micros()).await;
//                     #[cfg(feature = "rp2040")]
//                     Timer::delay(1500u64.micros()).await;
//                 }

//                 self.state = State::Standby1;

//                 config &= !(ConfigRegister::RxTxControl as u8);
//                 self.ce.set_high().map_err(Error::GpioError)?;
//                 self.write_register(0x00, &[config], spi).await?;
//                 #[cfg(feature = "systick")]
//                 Systick::delay(140u32.micros()).await;
//                 #[cfg(feature = "rp2040")]
//                 Timer::delay(140u64.micros()).await;

//                 self.state = State::Tx;
//             },
//             (State::Rx, State::Tx) => {
//                 self.ce.set_low().map_err(Error::GpioError)?;

//                 self.state = State::Standby1;

//                 config &= !(ConfigRegister::RxTxControl as u8);
//                 self.ce.set_high().map_err(Error::GpioError)?;
//                 self.write_register(0x00, &[config], spi).await?;
//                 #[cfg(feature = "systick")]
//                 Systick::delay(140u32.micros()).await;
//                 #[cfg(feature = "rp2040")]
//                 Timer::delay(140u64.micros()).await;

//                 self.state = State::Tx;
//             },
//             (State::Tx, State::Rx) => {
//                 self.ce.set_low().map_err(Error::GpioError)?;

//                 self.state = State::Standby1;

//                 config |= ConfigRegister::RxTxControl as u8;
//                 self.ce.set_high().map_err(Error::GpioError)?;
//                 self.write_register(0x00, &[config], spi).await?;
//                 #[cfg(feature = "systick")]
//                 Systick::delay(130u32.micros()).await;
//                 #[cfg(feature = "rp2040")]
//                 Timer::delay(130u64.micros()).await;

//                 self.state = State::Rx;
//             },
//             _ => (),
//         }

//         Ok(())
//     }

//     async fn write_full_config(&mut self, spi: &mut SPI) -> Result<(), (u8, Error<GPIOE, SPIE>)> {
//         // Write first 7 configs
//         for register in 0x00..=0x06 {
//             let mut config = [0u8];

//             self.read_register(register, &mut config, spi).await.map_err(|err| { (register, err) })?;
//             config[0] &= self.configuration.register_mask(register);
//             config[0] |= self.configuration.register_value(register);
//             self.write_register(register, &config, spi).await.map_err(|err| { (register, err) })?;

//             self.wait_for_ms(10).await;

//             let mut found_config = [0u8];
//             self.read_register(register, &mut found_config, spi).await.map_err(|err| { (register, err) })?;

//             if found_config[0] != config[0] {
//                 return Err((register, Error::UnableToConfigureRegister(register, config[0], found_config[0])));
//             }
//         }

//         // Write Base Register and Feature Registers
//         let base_config = self.configuration.pipe_configs[0].unwrap();
//         self.write_register(0x0A, base_config.address, spi).await.map_err(|err| { (0x0A, err) })?;

//         self.wait_for_ms(10).await;

//         let mut found_config = [0u8; 5];
//         self.read_register(0x0A, &mut found_config, spi).await.map_err(|err| { (0x0A, err)})?;

//         for i in 0..base_config.address.len() {
//             if base_config.address[i] != found_config[i] {
//                 return Err((0x0A, Error::UnableToConfigureRegister(0x0A, base_config.address[i], found_config[i])));
//             }
//         }

//         // Write Other Pipe Registers
//         let mut pipe_index = 1;
//         for register in 0x0B..=0x0F {
//             if let Some(pipe_config) = self.configuration.pipe_configs[pipe_index] {
//                 self.write_register(register, &[pipe_config.address[0]], spi).await.map_err(|err| { (register, err)})?;

//                 self.wait_for_ms(10).await;

//                 let mut found_config = [0u8];
//                 self.read_register(register, &mut found_config, spi).await.map_err(|err| { (register, err)})?;

//                 if found_config[0] != pipe_config.address[0] {
//                     return Err(Error::UnableToConfigureRegister(register, pipe_config.address[0], found_config[0])).map_err(|err| { (register, err)})?;
//                 }
//             } else {
//                 break;
//             }

//             pipe_index += 1;
//         }

//         // Write Transmit Register
//         self.write_register(0x10, self.configuration.tx_address, spi).await.map_err(|err| { (0x10, err)})?;

//         self.wait_for_ms(10).await;

//         let mut found_config = [0u8; 5];
//         self.read_register(0x10, &mut found_config, spi).await.map_err(|err| { (0x10, err)})?;

//         for i in 0..self.configuration.tx_address.len() {
//             if self.configuration.tx_address[i] != found_config[i] {
//                 return Err((0x10, Error::UnableToConfigureRegister(0x10, self.configuration.tx_address[i], found_config[i])));
//             }
//         }

//         // Write Address Registers
//         for register in (0x1C..=0x1D).rev() {
//             let mut activate_instruction = [Command::Activate.opcode(), 0x73];
//             self.safe_transfer_spi(spi, &mut activate_instruction).await.map_err(|err| { (register, err)})?;

//             let mut config = [0u8];

//             self.read_register(register, &mut config, spi).await.map_err(|err| { (register, err)})?;
//             config[0] &= self.configuration.register_mask(register);
//             config[0] |= self.configuration.register_value(register);
//             self.write_register(register, &config, spi).await.map_err(|err| { (register, err)})?;

//             self.wait_for_ms(10).await;

//             let mut found_config = [0u8];
//             self.read_register(register, &mut found_config, spi).await.map_err(|err| { (register, err)})?;

//             if found_config[0] != config[0] {
//                 return Err((register, Error::UnableToConfigureRegister(register, config[0], found_config[0])));
//             }
//         }

//         Ok(())
//     }

//     #[cfg(feature = "systick")]
//     async fn wait_for_ms(&self, ms: u32) {
//         Systick::delay(ms.millis()).await;
//     }

//     #[cfg(feature = "systick")]
//     async fn wait_for_us(&self, us: u32) {
//         Systick::delay(us.micros()).await;
//     }

//     #[cfg(feature = "rp2040")]
//     async fn wait_for_ms(&self, ms: u64) {
//         Timer::delay(ms.millis()).await;
//     }

//     #[cfg(feature = "rp2040")]
//     async fn wait_for_us(&self, us: u64) {
//         Timer::delay(ms.millis()).await;
//     }
// }