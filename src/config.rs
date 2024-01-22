//! Configuration of the NRF24L01 Module

use defmt::Format;

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

use crate::register::Register;

pub trait RegisterValue {
    fn register_value(&self, register: Register) -> Option<u8>;
}

/// The configuration of the NRF24L01 Module
#[derive(Debug, Clone, Copy, Format)]
pub struct Configuration<'a> {
    // The width of tx/rx addresses
    pub address_width: AddressWidth,
    // The level of bit correction
    pub crc: CRC,
    // The data rate of the NRF24L01 Module
    pub data_rate: DataRate,
    // The RF Channel Frequency [0, 124]
    pub rf_channel: u8,
    // The power amplifier level
    pub amplifier_level: PowerAmplifier,
    // The interrupt mask for the module
    pub interrupt_mask: InterruptMask,
    // Pipes
    pub pipe_configs: [DataPipeConfig<'a>; 6],
    // Address to send to
    pub tx_address: &'a [u8],
    // Retransmit Delay (delay = x * 250 + 250)
    pub retransmit_delay: u8,
    // Number of retransmits
    pub retransmit_count: u8,
    // True if the payload is dynamic
    pub dynamic_payload: bool,
}

impl<'a> RegisterValue for Configuration<'a> {
    fn register_value(&self, register: Register) -> Option<u8> {
        // TODO: Verify Correct Address Length of Pipe Addresses
        match register {
            Register::Config => {
                match (self.interrupt_mask.register_value(register), self.crc.register_value(register)) {
                    (Some(mask), Some(crc)) => Some(mask | crc),
                    (Some(mask), None) => Some(register.get_default() | mask),
                    (None, Some(crc)) => Some(register.get_default() | crc),
                    (None, None) => Some(register.get_default()),
                }
            },
            Register::EnableAutoAcknowledge | Register::EnabledRxAddresses => self.pipe_configs.register_value(register),
            Register::SetupAddressWidths => self.address_width.register_value(register),
            Register::SetupAutoRetransmit => {
                Some((self.retransmit_delay << 4) | self.retransmit_count)
            },
            Register::RfChannel => Some(self.rf_channel),
            Register::RfSetup => {
                match (self.data_rate.register_value(register), self.amplifier_level.register_value(register)) {
                    (Some(rate), Some(level)) => Some(rate | level),
                    (Some(rate), None) => Some(register.get_default() | rate),
                    (None, Some(level)) => Some(register.get_default() | level),
                    (None, None) => Some(register.get_default()),
                }
            },
            Register::Status => Some(0xFF),
            Register::RxAddressP2 |
            Register::RxAddressP3 |
            Register::RxAddressP4 |
            Register::RxAddressP5 => self.pipe_configs.register_value(register),
            Register::DynamicPayload => self.pipe_configs.register_value(register),
            _ => None,
        }
    }
}

impl<'a> Default for Configuration<'a> {
    fn default() -> Self {
        Self {
            data_rate: DataRate::R1Mb,
            address_width: AddressWidth::A5Bytes,
            crc: CRC::CRC1,
            rf_channel: 2,
            amplifier_level: PowerAmplifier::PAMax,
            interrupt_mask: InterruptMask::none(),
            pipe_configs: [
                DataPipeConfig::new(true, true, &[0xE7, 0xE7, 0xE7, 0xE7, 0xE7]),
                DataPipeConfig::new(true, true, &[0xC2, 0xC2, 0xC2, 0xC2, 0xC2]),
                DataPipeConfig::new(false, true, &[0xC3]),
                DataPipeConfig::new(false, true, &[0xC4]),
                DataPipeConfig::new(false, true, &[0xC5]),
                DataPipeConfig::new(false, true, &[0xC6]),
            ],
            tx_address: &[0xE7, 0xE7, 0xE7, 0xE7, 0xE7],
            retransmit_count: 3,
            retransmit_delay: 0,
            dynamic_payload: false,
        }
    }
}

// impl<'a> Configuration<'a> {
//     // Create a new nrf24l01 configuration
//     pub const fn new(
//         data_rate: DataRate,
//         rf_channel: u8,
//         amplifier_level: PowerAmplifier,
//         bit_correction: Option<BitCorrection>,
//         interrupt_mask: InterruptMask,
//         address_width: AddressWidth,
//         pipe_configs: [Option<DataPipeConfig<'a>>; 6],
//         tx_address: &'a [u8],
//         retransmit_delay: u8,
//         retransmit_count: u8,
//     ) -> Result<Self, ConfigurationError> {
//         if rf_channel > 124 {
//             return Err(ConfigurationError::InvalidRfChannel);
//         } else if retransmit_delay > 0xf {
//             return Err(ConfigurationError::InvalidTransmitDelay);
//         } else if retransmit_count > 0xf {
//             return Err(ConfigurationError::InvalidTransmitCount);
//         } else if tx_address.len() != address_width.as_u8() as usize {
//             return Err(ConfigurationError::InvalidTxAddressLength);
//         }

//         let mut dynamic = false;
//         let mut auto_ack = false;

//         // I want this to be constant so I need to do a bunch of work by hand
//         if let Some(config) = pipe_configs[0] {
//             if config.address.len() != address_width.as_u8() as usize {
//                 return Err(ConfigurationError::InvalidAddressLength);
//             }

//             if config.dynamic_payload {
//                 dynamic = true;
//             }

//             if config.auto_acknowledge {
//                 auto_ack = true;
//             }
//         } else {
//             return Err(ConfigurationError::MissingFirstPipeConfig);
//         }

//         if let Some(config) = pipe_configs[1] {
//             if config.address.len() != 1 {
//                 return Err(ConfigurationError::InvalidAddressLength);
//             }

//             if config.dynamic_payload {
//                 dynamic = true;
//             }

//             if config.auto_acknowledge {
//                 auto_ack = true;
//             }
//         }

//         if let Some(config) = pipe_configs[2] {
//             if config.address.len() != 1 {
//                 return Err(ConfigurationError::InvalidAddressLength);
//             }

//             if config.dynamic_payload {
//                 dynamic = true;
//             }

//             if config.auto_acknowledge {
//                 auto_ack = true;
//             }
//         }

//         if let Some(config) = pipe_configs[3] {
//             if config.address.len() != 1 {
//                 return Err(ConfigurationError::InvalidAddressLength);
//             }

//             if config.dynamic_payload {
//                 dynamic = true;
//             }

//             if config.auto_acknowledge {
//                 auto_ack = true;
//             }
//         }

//         if let Some(config) = pipe_configs[4] {
//             if config.address.len() != 1 {
//                 return Err(ConfigurationError::InvalidAddressLength);
//             }

//             if config.dynamic_payload {
//                 dynamic = true;
//             }

//             if config.auto_acknowledge {
//                 auto_ack = true;
//             }
//         }

//         if let Some(config) = pipe_configs[5] {
//             if config.address.len() != 1 {
//                 return Err(ConfigurationError::InvalidAddressLength);
//             }

//             if config.dynamic_payload {
//                 dynamic = true;
//             }

//             if config.auto_acknowledge {
//                 auto_ack = true;
//             }
//         }

//         Ok(Self {
//             data_rate,
//             rf_channel,
//             amplifier_level,
//             bit_correction,
//             interrupt_mask,
//             address_width,
//             pipe_configs,
//             tx_address,
//             retransmit_delay,
//             retransmit_count,
//             dynamic_payload: dynamic,
//             acknowledge_payload: auto_ack,
//             enable_dynamic_acknowledge: if auto_ack { true } else {false},
//         })
//     }
// }

// impl<'a> Default for Configuration<'a> {
//     // Default configuration for nRF24L01 Module
//     fn default() -> Self {
//         Self {
//             data_rate: DataRate::R250kb,
//             rf_channel: 0u8,
//             amplifier_level: PowerAmplifier::PALow,
//             bit_correction: Some(BitCorrection::CRC1),
//             interrupt_mask: InterruptMask::none(),
//             address_width: AddressWidth::A3Bytes,
//             pipe_configs: [Some(DataPipeConfig::default()); 6],
//             tx_address: b"000",
//             retransmit_delay: 0u8,
//             retransmit_count: 0u8,
//             dynamic_payload: true,
//             acknowledge_payload: true,
//             enable_dynamic_acknowledge: true,
//         }
//     }
// }

// impl<'a> RegisterValue for Configuration<'a> {
//     fn register_value(&self, address: u8) -> u8 {
//         match address {
//             0x00 => {
//                 if let Some(bit_correction) = self.bit_correction {
//                     return self.interrupt_mask.register_value(0x00) | 0b0000_1000 | bit_correction.register_value(0x00);
//                 }
//                 return self.interrupt_mask.register_value(0x00);
//             },
//             0x01 => self.pipe_configs.register_value(0x01),
//             0x02 => self.pipe_configs.register_value(0x02),
//             0x03 => self.address_width.register_value(0x03),
//             0x04 => (0xF0 & self.retransmit_delay) | (0x0F & self.retransmit_count),
//             0x05 => self.rf_channel,
//             0x06 => self.data_rate.register_value(0x06) | self.amplifier_level.register_value(0x06),
//             0x1C => self.pipe_configs.register_value(0x1C),
//             0x1D => {
//                 let mut value = 0u8;
                
//                 if self.dynamic_payload {
//                     value |= 0b0000_0100;
//                 }

//                 if self.enable_dynamic_acknowledge {
//                     value |= 0b0000_0010;
//                 }

//                 if self.dynamic_payload {
//                     value |= 0b0000_0001;
//                 }

//                 return value;
//             }
//             _ => todo!()
//         }
//     }
// }

// impl<'a> RegisterMask for Configuration<'a> {
//     fn register_mask(&self, address: u8) -> u8 {
//         match address {
//             0x00 => 0b1000_0011,
//             0x01 | 0x02 | 0x1C => 0b1100_0000,
//             0x03 => 0b1111_1100,
//             0x04 => 0b0000_0000,
//             0x05 => 0b1000_0000,
//             0x06 => 0b0101_0001,
//             0x1D => 0b1111_1000,
//             _ => 0b1111_1111,
//         }
//     }
// }

// /// Trait for all configurations to get their corresponding register value
// pub trait RegisterValue {
//     // Converts the configuration to its corresponding register value
//     fn register_value(&self, address: u8) -> u8;
// }

// /// Mask the configuration writable parts of the configuration u8
// pub trait RegisterMask {
//     fn register_mask(&self, address: u8) -> u8;
// }