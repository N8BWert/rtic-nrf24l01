//!
//! Configuration for the nRF24L01+ Module
//! 

use crate::register::Register;

pub mod address_width;
use address_width::AddressWidth;

pub mod crc;
use crc::CRC;

pub mod data_pipe;
use data_pipe::DataPipeConfig;

pub mod data_rate;
use data_rate::DataRate;

pub mod interrupt_mask;
use interrupt_mask::InterruptMask;

pub mod power_amplifier;
use power_amplifier::PowerAmplifier;

pub trait RegisterValue {
    /// A mask that isolates only the bits effected by the configuration option
    fn register_mask(&self, register: Register) -> u8;

    /// The bit value (pre-shifted) of the configuration option
    fn register_value(&self, register: Register) -> u8;
}

/// Configuration of the nRF24L01+ Module.  I will be keeping pipe 0 the same as the write address to enable auto acknowledge
/// to work properly.
pub struct Configuration {
    pub interrupt_mask: InterruptMask,
    pub crc: CRC,
    pub address_width: AddressWidth,
    pub retransmit_delay: u8,
    pub retransmit_count: u8,
    pub rf_channel: u8,
    pub data_rate: DataRate,
    pub power_amplifier: PowerAmplifier,
    pub tx_address: [u8; 5],
    pub tx_length: u8,
    pub network_mask: [u8; 4],
    pub receive_pipes: [DataPipeConfig; 5],
    pub dynamic_payload_enabled: bool,
    pub ack_payload_enabled: bool,
    pub dynamic_ack_enabled: bool,
}

impl RegisterValue for Configuration {
    fn register_mask(&self, register: Register) -> u8 {
        match register {
            Register::Config => !(0b0111_1100),
            Register::EnableAutoAcknowledge => !(0b0011_1111),
            Register::EnableRx => !(0b0011_1111),
            Register::SetupAddressWidth => !(0b0000_0011),
            Register::SetupRetransmit => !(0xFF),
            Register::RfChannel => !(0b0111_1111),
            Register::RfSetup => !(0b0010_1110),
            Register::Status => !(0b0111_0000),
            Register::ObserveTx => !(0x00),
            Register::ReceivedPower => !(0x00),
            Register::RxAddressP2 |
            Register::RxAddressP3 |
            Register::RxAddressP4 |
            Register::RxAddressP5 => !(0xFF),
            Register::RxPayloadWidthP0 |
            Register::RxPayloadWidthP1 |
            Register::RxPayloadWidthP2 |
            Register::RxPayloadWidthP3 |
            Register::RxPayloadWidthP4 |
            Register::RxPayloadWidthP5 => !(0b0011_1111),
            Register::FifoStatus => !(0x00),
            Register::DynamicPayload => !(0b0011_1111),
            Register::Feature => !(0b0000_0111),
            _ => 0xFF,
        }
    }

    fn register_value(&self, register: Register) -> u8 {
        match register {
            Register::Config => self.interrupt_mask.register_value(register) | self.crc.register_value(register),
            Register::EnableAutoAcknowledge => self.receive_pipes.register_value(register),
            Register::EnableRx => self.receive_pipes.register_value(register),
            Register::SetupAddressWidth => self.address_width.register_value(register),
            Register::RfChannel => self.rf_channel,
            Register::RfSetup => self.data_rate.register_value(register) | self.power_amplifier.register_value(register),
            Register::Status => 0xFF,
            Register::ObserveTx => 0x00,
            Register::ReceivedPower => 0x00,
            Register::RxAddressP2 |
            Register::RxAddressP3 |
            Register::RxAddressP4 |
            Register::RxAddressP5 => self.receive_pipes.register_value(register),
            Register::RxPayloadWidthP0 => self.tx_length,
            Register::RxPayloadWidthP1 |
            Register::RxPayloadWidthP2 |
            Register::RxPayloadWidthP3 |
            Register::RxPayloadWidthP4 |
            Register::RxPayloadWidthP5 => self.receive_pipes.register_value(register),
            Register::FifoStatus => 0x00,
            Register::DynamicPayload => if self.dynamic_payload_enabled { 0xFF } else { 0x00 },
            Register::Feature => {
                let mut value = 0;

                if self.dynamic_payload_enabled {
                    value |= 1 << 2;
                }

                if self.ack_payload_enabled {
                    value |= 1 << 1;
                }

                if self.dynamic_ack_enabled {
                    value |= 1;
                }

                value
            },
            _ => 0x00,
        }
    }
}