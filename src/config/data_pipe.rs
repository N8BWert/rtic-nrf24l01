//! Configuration for data pipes on the nRF24L01 module

use crate::register::Register;
use super::RegisterValue;

use defmt::Format;

/// The Configuration for a singular data pipe
#[derive(Debug, Clone, Copy, Format)]
pub struct DataPipeConfig<'a> {
    pub enabled: bool,
    pub auto_acknowledge: bool,
    pub address: &'a [u8],
}

impl<'a> DataPipeConfig<'a> {
    pub const fn new(
        enabled: bool,
        auto_acknowledge: bool,
        address: &'a [u8],
    ) -> Self {
        Self {
            enabled,
            auto_acknowledge,
            address,
        }
    }

    pub const fn disabled() -> Self {
        Self {
            enabled: false,
            auto_acknowledge: false,
            address: b"Node1",
        }
    }
}

impl<'a> Default for DataPipeConfig<'a> {
    fn default() -> Self {
        Self {
            enabled: false,
            auto_acknowledge: false,
            address: b"abc",
        }
    }
}

impl<'a> RegisterValue for [DataPipeConfig<'a>; 6] {
    fn register_value(&self, register: Register) -> Option<u8> {
        match register {
            Register::EnableAutoAcknowledge => {
                let mut value = 0u8;
                for (i, pipe) in self.iter().enumerate() {
                    if pipe.enabled && pipe.auto_acknowledge {
                        value |= 1 << i;
                    }
                }
                Some(value)
            },
            Register::EnabledRxAddresses => {
                let mut value = 0u8;
                for (i, pipe) in self.iter().enumerate() {
                    if pipe.enabled {
                        value |= 1 << i;
                    }
                }
                Some(value)
            },
            Register::RxAddressP2 => {
                if self[2].enabled {
                    Some(self[2].address[0])
                } else {
                    None
                }
            },
            Register::RxAddressP3 => {
                if self[3].enabled {
                    Some(self[3].address[0])
                } else {
                    None
                }
            },
            Register::RxAddressP4 => {
                if self[4].enabled {
                    Some(self[4].address[0])
                } else {
                    None
                }
            },
            Register::RxAddressP5 => {
                if self[5].enabled {
                    Some(self[5].address[0])
                } else {
                    None
                }
            },
            _ => None,
        }
    }
}