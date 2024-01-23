#![no_std]

use core::marker::PhantomData;

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
use register::Register;

// Datasheet defined time periods
const POWER_ON_RESET_MS: u32 = 100;
const START_UP_US: u32 = 1500;
const RX_SETUP_US: u32 = 130;
const TX_SETUP_US: u32 = 130;
const TX_SEND_TIME_US: u32 = 10;
const CE_DELAY_US: u32 = 4;

pub struct Radio<CE: OutputPin<Error=GPIOE>,
                 CSN: OutputPin<Error=GPIOE>,
                 SPI: Transfer<u8, Error=SPIE>,
                 DELAY: DelayMs<u32> + DelayUs<u32>,
                 GPIOE,
                 SPIE> {
    ce: CE,
    csn: CSN,
    configuration: Configuration,
    phantom: PhantomData<(SPI, DELAY)>,
}

impl<CE, CSN, SPI, DELAY, GPIOE, SPIE> Radio<CE, CSN, SPI, DELAY, GPIOE, SPIE>
    where CE: OutputPin<Error=GPIOE>,
          CSN: OutputPin<Error=GPIOE>,
          SPI: Transfer<u8, Error=SPIE>,
          DELAY: DelayMs<u32> + DelayUs<u32> {
    /// Create a new Radio Driver Instance
    pub fn new(mut ce: CE, mut csn: CSN, configuration: Configuration) -> Self {
        let _ = ce.set_low();
        let _ = csn.set_high();

        Self {
            ce,
            csn,
            configuration,
            phantom: PhantomData
        }
    }

    /// Send a packet
    pub fn send_packet(&mut self, packet: &[u8], spi: &mut SPI, delay: &mut DELAY) -> Result<bool, RadioError> {
        if packet.len() != self.configuration.tx_length as usize {
            return Err(RadioError::InvalidPayloadLength);
        }

        let mut command = [0u8; 33];
        command[0] = Command::WriteTxPayload.opcode();
        command[1..=packet.len()].copy_from_slice(&packet[..]);
        self.safe_transfer_spi(&mut command, spi);

        // Try to send the packet and wait for the packet to be received
        let _ = self.ce.set_high();
        delay.delay_us(TX_SEND_TIME_US + CE_DELAY_US + TX_SETUP_US);
        let mut sent = false;
        for _ in 0..=self.configuration.retransmit_count {
            let status = self.read_status(spi);
            if status & (1 << 5) != 0 {
                sent = true;
                break;
            } else {
                delay.delay_us((self.configuration.retransmit_delay as u32) * 250);
            }
        }
        let _ = self.ce.set_low();

        if sent {
            self.clear_interrupts(spi);
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Receive a packet from the Rx Fifo, returning the pipe it is from
    pub fn receive_packet(&mut self, buffer: &mut [u8], spi: &mut SPI) -> Result<u8, RadioError> {
        if !self.packet_ready(spi) {
            return Err(RadioError::NoPacketReady);
        }

        let status = self.read_status(spi);
        let pipe = (status & 0b0000_1110) >> 1;

        let mut read_buffer = [0u8; 33];
        read_buffer[0] = Command::ReadRxPayload.opcode();
        self.safe_transfer_spi(&mut read_buffer, spi);

        let buffer_length = buffer.len();
        buffer[..].copy_from_slice(&read_buffer[1..=buffer_length]);

        Ok(pipe)
    }

    /// Check whether there is a packet ready
    pub fn packet_ready(&mut self, spi: &mut SPI) -> bool {
        let status = self.read_status(spi);
        if status & (1 << 6) != 0 {
            self.clear_interrupts(spi);
            true
        } else {
            false
        }
    }

    /// Set the payload length of a given pipe
    pub fn set_pipe_payload_length(&mut self, pipe: u8, length: u8, spi: &mut SPI) {
        if pipe == 0 {
            self.configuration.tx_length = length;
        } else {
            self.configuration.receive_pipes[(pipe-1) as usize].payload_length = length;
        }

        self.write_byte_register(Register::payload_length_register(pipe), length, spi);
    }

    /// Set the network mask of the radio.
    pub fn set_network_mask(&mut self, network_mask: &[u8], spi: &mut SPI) -> Result<(), RadioError> {
        let address_width = self.configuration.address_width.as_usize();
        if network_mask.len() != address_width - 1 {
            return Err(RadioError::InvalidAddressLength);
        }

        self.configuration.network_mask[..(address_width-1)].copy_from_slice(&network_mask[..]);

        let mut rx_address = [0u8; 5];
        
        rx_address[..(address_width-1)].copy_from_slice(&network_mask[..]);
        rx_address[address_width-1] = self.configuration.receive_pipes[0].address;

        self.write_register(Register::RxAddressP1, &rx_address[..address_width], spi);

        Ok(())
    }

    /// Do not use this with pipe = 0 because that pipe is reserved for Tx acks
    pub fn set_pipe_address(&mut self, pipe: u8, address: u8, spi: &mut SPI) {
        match pipe {
            1 => {
                self.configuration.receive_pipes[(pipe - 1) as usize].address = address;

                let address_length = self.configuration.address_width.as_usize();
                let mut rx_address = [0u8; 5];
                rx_address[..(address_length-1)].copy_from_slice(&self.configuration.network_mask[..]);
                rx_address[address_length-1] = address;
                self.write_register(Register::RxAddressP1, &rx_address, spi);
            },
            2..=5 => {
                self.configuration.receive_pipes[(pipe - 1) as usize].address = address;
                self.write_byte_register(Register::address_register(pipe), address, spi);
            },
            _ => (),
        }
    }

    /// Set the length of the radio's address width
    pub fn set_address_width(&mut self, address_width: AddressWidth, spi: &mut SPI) {
        if address_width != self.configuration.address_width {
            // Change the radio register for address width
            self.configuration.address_width = address_width;
            let mut register_value = self.read_byte_register(Register::SetupAddressWidth, spi);
            register_value &= address_width.register_mask(Register::SetupAddressWidth);
            register_value |= address_width.register_value(Register::SetupAddressWidth);
            self.write_byte_register(Register::SetupAddressWidth, register_value, spi);

            // Change the configuration values for network mask and tx_address
            let address_width = address_width.as_usize();
            self.configuration.tx_address[address_width - 1] = 0x00;
            self.configuration.network_mask[address_width - 2] = 0x00;

            // Write the tx and read addresses to the radio
            let tx_address = self.configuration.tx_address;
            self.write_register(Register::TxAddress, &tx_address[..address_width], spi);
            self.write_register(Register::RxAddressP0, &tx_address[..address_width], spi);

            // Write new network mask
            let mut rx_address = [0u8; 5];
            rx_address[..(address_width-1)].copy_from_slice(&self.configuration.network_mask[..(address_width-1)]);
            rx_address[address_width-1] = self.configuration.receive_pipes[0].address;
            self.write_register(Register::RxAddressP1, &rx_address[..address_width], spi);
        }
    }

    /// Put the radio into a power up mode.
    pub fn power_up(&mut self, spi: &mut SPI, delay: &mut DELAY) {
        let mut config = self.read_config(spi);
        config |= 1 << 1;
        self.write_byte_register(Register::Config, config, spi);

        delay.delay_us(START_UP_US);
    }

    /// Put the radio into a powered down mode.
    pub fn power_down(&mut self, spi: &mut SPI, delay: &mut DELAY) {
        let mut config = self.read_config(spi);
        config &= !(1 << 1);
        self.write_byte_register(Register::Config, config, spi);

        delay.delay_us(START_UP_US);
    }

    /// Put the radio into RX mode an listen for incoming packets
    pub fn start_listening(&mut self, spi: &mut SPI, delay: &mut DELAY) {
        let mut config = self.read_config(spi);
        config |= 1;
        self.write_byte_register(Register::Config, config, spi);

        delay.delay_us(RX_SETUP_US);
    }

    /// Take the radio out of Rx mode (likely in preparation for transmission)
    pub fn stop_listening(&mut self, spi: &mut SPI, delay: &mut DELAY) {
        let mut config = self.read_config(spi);
        config &= !(1);
        self.write_byte_register(Register::Config, config, spi);

        delay.delay_us(RX_SETUP_US);
    }

    /// For writing to work, we need to ensure that the TxAddress is set and the TxAddress0
    /// is set to the same value
    pub fn open_new_writing_pipe(&mut self, address: &[u8], spi: &mut SPI) -> Result<(), RadioError> {
        let address_width = self.configuration.address_width.as_usize();
        if address.len() != address_width {
            return Err(RadioError::InvalidAddressLength);
        }

        self.configuration.tx_address[..address_width].copy_from_slice(&address[..]);

        self.write_register(Register::TxAddress, address, spi);
        self.write_register(Register::RxAddressP0, address, spi);
        self.open_reading_pipe(0, spi);

        Ok(())
    }

    /// Open the previously set writing pipe
    pub fn open_writing_pipe(&mut self, spi: &mut SPI) {
        self.open_reading_pipe(0, spi);
    }

    /// Close the rx pipe associated with the transmission.
    pub fn close_writing_pipe(&mut self, spi: &mut SPI) {
        self.close_reading_pipe(0, spi);
    }

    /// Open a given rx pipe
    pub fn open_reading_pipe(&mut self, pipe: u8, spi: &mut SPI) {
        let mut open_pipes = self.read_byte_register(Register::EnableRx, spi);
        open_pipes |= 1 << pipe;
        self.write_byte_register(Register::EnableRx, open_pipes, spi);
    }

    /// Close a given rx pipe
    pub fn close_reading_pipe(&mut self, pipe: u8, spi: &mut SPI) {
        let mut open_pipes = self.read_byte_register(Register::EnableRx, spi);
        open_pipes &= !(1 << pipe);
        self.write_byte_register(Register::EnableRx, open_pipes, spi);
    }

    /// Apply the configuration given to the driver and return true if the radio is configured correctly.
    pub fn begin(&mut self, spi: &mut SPI, delay: &mut DELAY) -> Result<(), RadioError> {
        delay.delay_ms(POWER_ON_RESET_MS);

        for register in Register::write_registers() {
            match register {
                Register::RxAddressP0 | Register::TxAddress => {
                    let address_width = self.configuration.address_width.as_usize();
                    let rx_address = self.configuration.tx_address;

                    self.write_register(register, &rx_address[..address_width], spi);
                    delay.delay_ms(5);
                    let mut address_buffer = [0u8; 5];
                    self.read_register(register, &mut address_buffer, spi);

                    for i in 0..address_width {
                        if self.configuration.tx_address[i] != address_buffer[i] {
                            return Err(RadioError::UnableToWriteToRegister { register, expected: self.configuration.tx_address[i], found: address_buffer[i] });
                        }
                    }
                },
                Register::RxAddressP1 => {
                    let address_width = self.configuration.address_width.as_usize();
                    let mut rx_address = [0u8; 5];
                    rx_address[..(address_width-1)].copy_from_slice(&self.configuration.network_mask[..(address_width-1)]);
                    rx_address[address_width-1] = self.configuration.receive_pipes[0].address;
                    
                    self.write_register(register, &rx_address[..address_width], spi);
                    delay.delay_ms(5);
                    let mut address_buffer = [0u8; 5];
                    self.read_register(register, &mut address_buffer, spi);

                    for i in 0..address_width {
                        if rx_address[i] != address_buffer[i] {
                            return Err(RadioError::UnableToWriteToRegister { register, expected: rx_address[i], found: address_buffer[i] });
                        }
                    }
                },
                _ => {
                    let mut address_value = self.read_byte_register(register, spi);
                    address_value &= self.configuration.register_mask(register);
                    address_value |= self.configuration.register_value(register);

                    self.write_byte_register(register, address_value, spi);
                    delay.delay_ms(5);
                    let found = self.read_byte_register(register, spi) & self.configuration.register_mask(register);
                    
                    if found != self.configuration.register_value(register) {
                        return Err(RadioError::UnableToWriteToRegister { register, expected: self.configuration.register_value(register), found });
                    }
                },
            }

            delay.delay_ms(5);
        }

        Ok(())
    }

    /// Reset the radio to it's default state (theoretically, this should be called every time the radio is
    /// so that a defined state is available.  However, begin should (in theory) do the same thing).
    pub fn reset(&mut self, spi: &mut SPI, delay: &mut DELAY) {
        delay.delay_ms(POWER_ON_RESET_MS);

        for register in Register::write_registers() {
            match register {
                Register::RxAddressP0 => self.write_register(register, &Register::default_rx_0(), spi),
                Register::RxAddressP1 => self.write_register(register, &Register::default_rx_1(), spi),
                Register::TxAddress => self.write_register(register, &Register::default_tx(), spi),
                _ => self.write_byte_register(register, register.default_value(), spi),
            }

            delay.delay_ms(5);
        }
    }

    pub fn flush_tx(&mut self, spi: &mut SPI) {
        let mut command = [Command::FlushTx.opcode()];
        self.safe_transfer_spi(&mut command, spi);
    }

    pub fn flush_rx(&mut self, spi: &mut SPI) {
        let mut command = [Command::FlushRx.opcode()];
        self.safe_transfer_spi(&mut command, spi);
    }

    pub fn read_config(&mut self, spi: &mut SPI) -> u8 {
        self.read_byte_register(Register::Config, spi)
    }

    pub fn read_status(&mut self, spi: &mut SPI) -> u8 {
        let mut command = [Command::Nop.opcode()];
        self.safe_transfer_spi(&mut command, spi);
        command[0]
    }

    pub fn read_fifo_status(&mut self, spi: &mut SPI) -> u8 {
        self.read_byte_register(Register::FifoStatus, spi)
    }

    pub fn clear_interrupts(&mut self, spi: &mut SPI) {
        self.write_byte_register(Register::Status, 0xFF, spi);
    }
    
    fn write_byte_register(&mut self, register: Register, data: u8, spi: &mut SPI) {
        let mut command = [Command::WriteRegister(register).opcode(), data];
        self.safe_transfer_spi(&mut command, spi);
    }

    fn write_register(&mut self, register: Register, data: &[u8], spi: &mut SPI) {
        let mut command = [0u8; 6];
        command[0] = Command::WriteRegister(register).opcode();
        command[1..=data.len()].copy_from_slice(&data[..]);
        self.safe_transfer_spi(&mut command[..=data.len()], spi);
    }

    fn read_byte_register(&mut self, register: Register, spi: &mut SPI) -> u8 {
        let mut command = [Command::ReadRegister(register).opcode(), 0x00];
        self.safe_transfer_spi(&mut command, spi);
        command[1]
    }

    fn read_register(&mut self, register: Register, buffer: &mut [u8], spi: &mut SPI) {
        let buffer_length = buffer.len();
        let mut command = [0x00; 6];
        command[0] = Command::ReadRegister(register).opcode();
        self.safe_transfer_spi(&mut command, spi);
        buffer[..].copy_from_slice(&command[1..=buffer_length]);
    }

    fn safe_transfer_spi(&mut self, data: &mut [u8], spi: &mut SPI) {
        let _ = self.csn.set_low();
        let mut res = spi.transfer(data);
        while res.is_err() {
            res = spi.transfer(data);
        }
        let _ = self.csn.set_high();
    }
}