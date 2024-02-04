//!
//! Driver for the nRF24L01+ Radio Module (based on the common RF24 C++ library)
//! 

#![no_std]

use core::marker::PhantomData;
use core::cmp::{min, max};

use config::data_rate::DataRate;
use config::{address_width, data_rate, power_amplifier};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::spi::Transfer;
use embedded_hal::blocking::delay::{DelayMs, DelayUs};

pub mod command;
use command::Command;

pub mod config;
use config::{Configuration, RegisterValue, address_width::AddressWidth};

pub mod error;
use error::RadioError;

pub mod register;
use register::{Register, RegisterMap};

// Datasheet defined time periods
const POWER_ON_RESET_MS: u32 = 100;
const START_UP_US: u32 = 1500;
const TX_SETUP_US: u32 = 130;
const TX_DELAY: u32 = 130;

const PWR_UP: u8 = 1;

const PRIM_RX: u8 = 0;

const RX_DR: u8 = 6;
const TX_DS: u8 = 5;
const MAX_RT: u8 = 4;

fn bit_value(value: u8) -> u8 {
    1 << value
}

pub struct Radio<CE: OutputPin<Error=GPIOE>,
                 CSN: OutputPin<Error=GPIOE>,
                 SPI: Transfer<u8, Error=SPIE>,
                 DELAY: DelayMs<u32> + DelayUs<u32>,
                 GPIOE,
                 SPIE> {
    ce: CE,
    csn: CSN,
    ack_payloads_enabled: bool,
    address_width: u8,
    dynamic_payloads_enabled: bool,
    status: u8,
    payload_size: u8,
    pipe0_reading_address: [u8; 5],
    config_register: u8,
    is_p_variant: bool,
    is_p0_rx: bool,
    phantom: PhantomData<(SPI, DELAY)>,
}

impl<CE, CSN, SPI, DELAY, GPIOE, SPIE> Radio<CE, CSN, SPI, DELAY, GPIOE, SPIE>
    where CE: OutputPin<Error=GPIOE>,
          CSN: OutputPin<Error=GPIOE>,
          SPI: Transfer<u8, Error=SPIE>,
          DELAY: DelayMs<u32> + DelayUs<u32> {
    /// Create a new Radio Driver Instance
    pub fn new(mut ce: CE, mut csn: CSN) -> Self {
        let _ = ce.set_low();
        let _ = csn.set_high();

        Self {
            ce,
            csn,
            payload_size: 0,
            address_width: 5,
            ack_payloads_enabled: false,
            config_register: 0,
            status: 0,
            is_p0_rx: false,
            is_p_variant: false,
            dynamic_payloads_enabled: false,
            pipe0_reading_address: [0u8; 5],
            phantom: PhantomData
        }
    }

    pub fn write(&mut self, buffer: &[u8], spi: &mut SPI, delay: &mut DELAY) -> bool {
        self.start_fast_write(buffer, false, spi, delay);

        while (self.read_status(spi, delay) & (bit_value(TX_DS) | bit_value(MAX_RT))) == 0 {
            delay.delay_us(100);
        }

        let _ = self.ce.set_low();

        self.write_byte_register(Register::Status, 0b0111_0000, spi, delay);

        if (self.status & bit_value(MAX_RT)) != 0 {
            self.flush_tx(spi, delay);
            return false;
        }
        return true;
    }

    pub fn start_fast_write(&mut self, buffer: &[u8], multicast: bool, spi: &mut SPI, delay: &mut DELAY) {
        self.write_payload(buffer, if multicast { Command::WriteTxPayloadNoAck } else { Command::WriteTxPayload }, spi, delay);
        let _ = self.ce.set_high();
    }

    pub fn write_payload(&mut self, buffer: &[u8], write_command: Command, spi: &mut SPI, delay: &mut DELAY) {
        let mut write_buffer = [0xFF; 33];
        write_buffer[0] = write_command.opcode();
        write_buffer[1..=buffer.len()].copy_from_slice(&buffer[..]);
        self.safe_transfer_spi(&mut write_buffer[..=buffer.len()], spi, delay);
        self.status = write_buffer[0]
    }

    pub fn read(&mut self, buffer: &mut [u8], spi: &mut SPI, delay: &mut DELAY) {
        self.read_payload(buffer, spi, delay);

        self.write_byte_register(Register::Status, bit_value(RX_DR), spi, delay);
    }

    pub fn read_payload(&mut self, buffer: &mut [u8], spi: &mut SPI, delay: &mut DELAY) {
        let mut read_buffer = [0xFF; 33];
        read_buffer[0] = Command::ReadRxPayload.opcode();
        self.safe_transfer_spi(&mut read_buffer[..=buffer.len()], spi, delay);
        for i in 0..buffer.len() {
            buffer[i] = read_buffer[i+1];
        }
        self.status = read_buffer[0];
    }

    /// Receive a packet from the Rx Fifo, returning the pipe it is from
    pub fn receive_packet(&mut self, buffer: &mut [u8], spi: &mut SPI, delay: &mut DELAY) -> Result<u8, RadioError> {
        if !self.packet_ready(spi, delay) {
            return Err(RadioError::NoPacketReady);
        }

        let status = self.read_status(spi, delay);
        let pipe = (status & 0b0000_1110) >> 1;

        let mut read_buffer = [0u8; 33];
        read_buffer[0] = Command::ReadRxPayload.opcode();
        self.safe_transfer_spi(&mut read_buffer, spi, delay);

        let buffer_length = buffer.len();
        buffer[..].copy_from_slice(&read_buffer[1..=buffer_length]);

        Ok(pipe)
    }

    /// Check whether there is a packet ready
    pub fn packet_ready(&mut self, spi: &mut SPI, delay: &mut DELAY) -> bool {
        let status = self.read_status(spi, delay);
        if status & (1 << 6) != 0 {
            self.clear_interrupts(spi, delay);
            true
        } else {
            false
        }
    }

    pub fn get_payload_size(&self) -> u8 {
        self.payload_size
    }

    /// Set the length of the radio's address width
    pub fn set_address_width(&mut self, address_width: AddressWidth, spi: &mut SPI, delay: &mut DELAY) {
        self.write_byte_register(Register::SetupAddressWidth, address_width.register_value(Register::SetupAddressWidth), spi, delay);
        self.address_width = address_width.as_usize() as u8;
    }

    /// Put the radio into a power up mode.
    pub fn power_up(&mut self, spi: &mut SPI, delay: &mut DELAY) {
        if (self.config_register & bit_value(PWR_UP)) == 0 {
            self.config_register |= bit_value(PWR_UP);
            self.write_byte_register(Register::Config, self.config_register, spi, delay);
            delay.delay_ms(START_UP_US);
        }
    }

    /// Put the radio into a powered down mode.
    pub fn power_down(&mut self, spi: &mut SPI, delay: &mut DELAY) {
        let mut config = self.read_config(spi, delay);
        config &= !(1 << 1);
        self.write_byte_register(Register::Config, config, spi, delay);

        delay.delay_us(START_UP_US);
    }

    /// Put the radio into RX mode an listen for incoming packets
    pub fn start_listening(&mut self, spi: &mut SPI, delay: &mut DELAY) {
        self.power_up(spi, delay);

        self.config_register |= bit_value(PRIM_RX);
        self.write_byte_register(Register::Config, self.config_register, spi, delay);
        self.write_byte_register(Register::Status, 0b0111_0000, spi, delay);
        let _ = self.ce.set_high();

        if self.is_p0_rx {
            let address = self.pipe0_reading_address;
            self.write_register(Register::RxAddressP0, &address, spi, delay);
        } else {
            self.close_reading_pipe(0, spi, delay);
        }
    }

    pub fn stop_listening(&mut self, spi: &mut SPI, delay: &mut DELAY) {
        let _ = self.ce.set_low();

        delay.delay_us(TX_DELAY);
        if self.ack_payloads_enabled {
            self.flush_tx(spi, delay);
        }

        self.config_register = self.config_register & !(bit_value(PRIM_RX));
        self.write_byte_register(Register::Config, self.config_register, spi, delay);

        let enabled_pipes = self.read_byte_register(Register::EnableRx, spi, delay) | 1;
        self.write_byte_register(Register::EnableRx, enabled_pipes, spi, delay);
    }

    /// Open the previously set writing pipe
    pub fn open_writing_pipe(&mut self, address: [u8; 5], spi: &mut SPI, delay: &mut DELAY) {
        self.write_register(Register::RxAddressP0, &address, spi, delay);
        self.write_register(Register::TxAddress, &address, spi, delay);
    }

    pub fn set_payload_size(&mut self, size: u8, spi: &mut SPI, delay: &mut DELAY) {
        self.payload_size = max(1, min(32, size));

        self.write_byte_register(Register::RxPayloadWidthP0, self.payload_size, spi, delay);
        self.write_byte_register(Register::RxPayloadWidthP1, self.payload_size, spi, delay);
        self.write_byte_register(Register::RxPayloadWidthP2, self.payload_size, spi, delay);
        self.write_byte_register(Register::RxPayloadWidthP3, self.payload_size, spi, delay);
        self.write_byte_register(Register::RxPayloadWidthP4, self.payload_size, spi, delay);
        self.write_byte_register(Register::RxPayloadWidthP5, self.payload_size, spi, delay);
    }

    pub fn set_retries(&mut self, delay: u8, count: u8, spi: &mut SPI, d: &mut DELAY) {
        self.write_byte_register(Register::SetupRetransmit, 
            min(15, delay) << 4 | min(15, count), spi, d);
    }

    pub fn open_reading_pipe(&mut self, pipe: u8, address: [u8; 5], spi: &mut SPI, delay: &mut DELAY) {
        if pipe == 0 {
            self.is_p0_rx = true;
        } else if pipe <= 5{
            if pipe < 2 {
                self.write_register(Register::RxAddressP1, &address, spi, delay);
            } else {
                self.write_byte_register(Register::address_register(pipe), address[0], spi, delay);
            }

            let enabled_pipes = self.read_byte_register(Register::EnableRx, spi, delay) | (1 << pipe);
            self.write_byte_register(Register::EnableRx, enabled_pipes, spi, delay);
        }
    }

    /// Close a given rx pipe
    pub fn close_reading_pipe(&mut self, pipe: u8, spi: &mut SPI, delay: &mut DELAY) {
        let mut open_pipes = self.read_byte_register(Register::EnableRx, spi, delay) & !bit_value(pipe);
        self.write_byte_register(Register::EnableRx, open_pipes, spi, delay);
        if pipe == 0 {
            self.is_p0_rx = false;
        }
    }

    /// Apply the configuration given to the driver and return true if the radio is configured correctly.
    pub fn begin(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), RadioError> {
        self.init_pins(delay);
        self.init_radio(spi, delay)
    }

    fn init_pins(&mut self, delay: &mut DELAY) {
        let _ = self.ce.set_low();
        let _ = self.csn.set_high();
        delay.delay_ms(100);
    }

    fn init_radio(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), RadioError> {
        delay.delay_ms(5);

        self.set_retries(5, 15, spi, delay);

        self.set_data_rate(data_rate::DataRate::R1Mb, spi, delay);

        let before_toggle = self.read_byte_register(Register::Feature, spi, delay);
        self.toggle_features(spi, delay);
        let after_toggle = self.read_byte_register(Register::Feature, spi, delay);
        self.is_p_variant = before_toggle == after_toggle;
        if after_toggle != 0 {
            if self.is_p_variant {
                self.toggle_features(spi, delay);
            }
            self.write_byte_register(Register::Feature, 0, spi, delay);
        }
        self.ack_payloads_enabled = false;
        self.write_byte_register(Register::DynamicPayload, 0, spi, delay);
        self.dynamic_payloads_enabled = false;
        self.write_byte_register(Register::EnableAutoAcknowledge, 0x3F, spi, delay);
        self.write_byte_register(Register::EnableRx, 3, spi, delay);
        self.set_payload_size(32, spi, delay);
        self.set_address_width(address_width::AddressWidth::A5Bytes, spi, delay);

        self.set_channel(76, spi, delay);

        self.write_byte_register(Register::Status, 0b0111_0000, spi, delay);

        self.flush_rx(spi, delay);
        self.flush_tx(spi, delay);

        self.write_byte_register(Register::Config, 0b0000_1100, spi, delay);
        self.config_register = self.read_byte_register(Register::Config, spi, delay);

        self.power_up(spi, delay);

        if self.config_register != 0b0000_1110 {
            Err(RadioError::BadConfiguration)
        } else {
            Ok(())
        }
    }

    pub fn toggle_features(&mut self, spi: &mut SPI, delay: &mut DELAY) {
        let mut buffer = [Command::Activate.opcode()];
        self.safe_transfer_spi(&mut buffer, spi, delay);
        self.status = buffer[0];
    }

    pub fn available(&mut self, spi: &mut SPI, delay: &mut DELAY) -> bool {
        let fifo_status = self.read_byte_register(Register::FifoStatus, spi, delay);

        return fifo_status & 1 == 0;
    }

    pub fn available_pipe(&mut self, spi: &mut SPI, delay: &mut DELAY) -> u8 {
        let pipe = (self.read_status(spi, delay) >> 1) & 0x07;
        pipe
    }

    /// Reset the radio to it's default state (theoretically, this should be called every time the radio is
    /// so that a defined state is available.  However, begin should (in theory) do the same thing).
    pub fn reset(&mut self, spi: &mut SPI, delay: &mut DELAY) {
        delay.delay_ms(POWER_ON_RESET_MS);

        for register in Register::write_registers() {
            match register {
                Register::RxAddressP0 => self.write_register(register, &Register::default_rx_0(), spi, delay),
                Register::RxAddressP1 => self.write_register(register, &Register::default_rx_1(), spi, delay),
                Register::TxAddress => self.write_register(register, &Register::default_tx(), spi, delay),
                _ => self.write_byte_register(register, register.default_value(), spi, delay),
            }

            delay.delay_ms(5);
        }
    }

    pub fn flush_tx(&mut self, spi: &mut SPI, delay: &mut DELAY) {
        let mut command = [Command::FlushTx.opcode(), 0xFF];
        self.safe_transfer_spi(&mut command, spi, delay);
    }

    pub fn flush_rx(&mut self, spi: &mut SPI, delay: &mut DELAY) {
        let mut command = [Command::FlushRx.opcode(), 0xFF];
        self.safe_transfer_spi(&mut command, spi, delay);
    }

    pub fn read_config(&mut self, spi: &mut SPI, delay: &mut DELAY) -> u8 {
        self.read_byte_register(Register::Config, spi, delay)
    }

    pub fn read_status(&mut self, spi: &mut SPI, delay: &mut DELAY) -> u8 {
        let mut command = [Command::Nop.opcode()];
        self.safe_transfer_spi(&mut command, spi, delay);
        self.status = command[0];
        self.status
    }

    pub fn read_fifo_status(&mut self, spi: &mut SPI, delay: &mut DELAY) -> u8 {
        self.read_byte_register(Register::FifoStatus, spi, delay)
    }

    pub fn clear_interrupts(&mut self, spi: &mut SPI, delay: &mut DELAY) {
        self.write_byte_register(Register::Status, 0xFF, spi, delay);
    }

    pub fn is_rx(&mut self, spi: &mut SPI, delay: &mut DELAY) -> bool {
        let config = self.read_config(spi, delay);
        if config & 0b11 == 0b11{
            true
        } else {
            false
        }
    }

    pub fn is_tx(&mut self, spi: &mut SPI, delay: &mut DELAY) -> bool {
        let config = self.read_config(spi, delay);
        if config & 0b11 == 0b10 {
            true
        } else {
            false
        }
    }

    pub fn pulse_ce(&mut self, delay: &mut DELAY) {
        let _ = self.ce.set_high();
        delay.delay_ms(1000);
        let _ = self.ce.set_low();
    }

    pub fn set_pa_level(&mut self, level: power_amplifier::PowerAmplifier, spi: &mut SPI, delay: &mut DELAY) {
        let mut setup = self.read_byte_register(Register::RfSetup, spi, delay) & 0xF8;
        setup |= level.register_value(Register::RfSetup);
        self.write_byte_register(Register::RfSetup, setup, spi, delay);
    }

    pub fn set_channel(&mut self, channel: u8, spi: &mut SPI, delay: &mut DELAY) {
        self.write_byte_register(Register::RfChannel, min(channel, 125), spi, delay);
    }

    pub fn set_data_rate(&mut self, data_rate: DataRate, spi: &mut SPI, delay: &mut DELAY) -> bool {
        let mut result = false;
        let mut setup = self.read_byte_register(Register::RfSetup, spi, delay);

        setup = setup & !(bit_value(5) | bit_value(3));
        setup |= data_rate.register_value(Register::RfSetup);

        self.write_byte_register(Register::RfSetup, setup, spi, delay);

        if self.read_byte_register(Register::RfSetup, spi, delay) != setup {
            result = true;
        }
        return result;
    }

    // Debugging function that returns a debug printable mapping of the device's internal registers
    pub fn get_registers(&mut self, spi: &mut SPI, delay: &mut DELAY) -> RegisterMap {
        let mut register_map = RegisterMap::default();

        for register in Register::all() {
            match register {
                Register::RxAddressP0 |
                Register::RxAddressP1 |
                Register::TxAddress => {
                    let mut register_buffer = [0u8; 5];
                    self.read_register(register, &mut register_buffer, spi, delay);
                    register_map.add_array_value(register, register_buffer);
                },
                _ => {
                    let register_value = self.read_byte_register(register, spi, delay);
                    register_map.add_register_value(register, register_value);
                },
            }
        }

        register_map
    }
    
    pub fn write_byte_register(&mut self, register: Register, data: u8, spi: &mut SPI, delay: &mut DELAY) {
        let mut command = [Command::WriteRegister(register).opcode(), data];
        self.safe_transfer_spi(&mut command, spi, delay);
        self.status = command[0];
    }

    fn write_register(&mut self, register: Register, data: &[u8], spi: &mut SPI, delay: &mut DELAY) {
        let mut command = [0xFFu8; 6];
        command[0] = Command::WriteRegister(register).opcode();
        command[1..=data.len()].copy_from_slice(&data[..]);
        self.status = command[0];
        self.safe_transfer_spi(&mut command[..=data.len()], spi, delay);
    }

    pub fn read_byte_register(&mut self, register: Register, spi: &mut SPI, delay: &mut DELAY) -> u8 {
        let mut command = [Command::ReadRegister(register).opcode(), 0xFF];
        self.safe_transfer_spi(&mut command, spi, delay);
        self.status = command[0];
        command[1]
    }

    pub fn read_register(&mut self, register: Register, buffer: &mut [u8], spi: &mut SPI, delay: &mut DELAY) {
        let buffer_length = buffer.len();
        let mut command = [0xFF; 6];
        command[0] = Command::ReadRegister(register).opcode();
        self.safe_transfer_spi(&mut command, spi, delay);
        self.status = command[0];
        buffer[..].copy_from_slice(&command[1..=buffer_length]);
    }

    fn safe_transfer_spi(&mut self, data: &mut [u8], spi: &mut SPI, delay: &mut DELAY) {
        let _ = self.csn.set_low();
        let _ = spi.transfer(data);
        let _ = self.csn.set_high();
        delay.delay_us(10);
        // let _ = self.csn.set_low();

        // let _ = self.csn.set_low();
        // let mut res = spi.transfer(data);
        // let _ = self.csn.set_high();
        // while res.is_err() {
        //     let _ = self.csn.set_low();
        //     res = spi.transfer(data);
        //     let _ = self.csn.set_high();
        // }
    }
}