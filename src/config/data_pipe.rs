//! Configuration for data pipes on the nRF24L01 module

use crate::register::Register;

use super::RegisterValue;

use defmt::Format;

/// The Configuration for a singular data pipe
#[derive(Debug, Clone, Copy, Format)]
pub struct DataPipeConfig {
    pub enabled: bool,
    pub auto_acknowledge: bool,
    pub address: u8,
}

impl DataPipeConfig {
    pub fn default_for_pipe(pipe: u8) -> Self {
        match pipe {
            1 => Self {
                enabled: true,
                auto_acknowledge: true,
                address: 0xC2,
            },
            2 => Self {
                enabled: false,
                auto_acknowledge: true,
                address: 0xC3,
            },
            3 => Self {
                enabled: false,
                auto_acknowledge: true,
                address: 0xC4,
            },
            4 => Self {
                enabled: false,
                auto_acknowledge: true,
                address: 0xC5,
            },
            5 => Self {
                enabled: false,
                auto_acknowledge: true,
                address: 0xC6,
            },
            _ => Self::disabled(),
        }
    }

    pub fn disabled() -> Self {
        Self {
            enabled: false,
            auto_acknowledge: false,
            address: 0,
        }
    }
}


impl RegisterValue for [DataPipeConfig; 5] {
    fn register_mask(&self, register: Register) -> u8 {
        match register {
            Register::EnableAutoAcknowledge => !(0b0011_1111),
            Register::EnableRx => !(0b0011_1111),
            Register::RxAddressP2 |
            Register::RxAddressP3 |
            Register::RxAddressP4 |
            Register::RxAddressP5 => !(0xFF),
            Register::DynamicPayload => !(0b0011_1111),
            _ => 0,
        }
    }

    fn register_value(&self, register: Register) -> u8 {
        match register {
            Register::EnableAutoAcknowledge => {
                let mut value = 1;
                for i in 0..5 {
                    if self[i].auto_acknowledge {
                        value |= 1 << (i+1);
                    }
                }
                value
            },
            Register::EnableRx => {
                let mut value = 1;
                for i in 0..5 {
                    if self[i].enabled {
                        value |= 1 << (i+1);
                    }
                }
                value
            },
            Register::RxAddressP2 => self[1].address,
            Register::RxAddressP3 => self[2].address,
            Register::RxAddressP4 => self[3].address,
            Register::RxAddressP5 => self[4].address,
            _ => 0
        }
    }
}
