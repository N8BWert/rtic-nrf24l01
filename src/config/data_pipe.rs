//! Configuration for data pipes on the nRF24L01 module

use super::RegisterValue;

use defmt::Format;

/// The Configuration for a singular data pipe
#[derive(Debug, Clone, Copy, Format)]
pub struct DataPipeConfig<'a> {
    pub enabled: bool,
    pub auto_acknowledge: bool,
    pub dynamic_payload: bool,
    pub address: &'a [u8],
}

impl<'a> DataPipeConfig<'a> {
    pub const fn new(
        enabled: bool,
        auto_acknowledge: bool,
        dynamic_payload: bool,
        address: &'a [u8],
    ) -> Self {
        Self {
            enabled,
            auto_acknowledge,
            dynamic_payload,
            address,
        }
    }
}

impl<'a> Default for DataPipeConfig<'a> {
    fn default() -> Self {
        Self {
            enabled: false,
            auto_acknowledge: false,
            dynamic_payload: false,
            address: b"000",
        }
    }
}

impl<'a> RegisterValue for [Option<DataPipeConfig<'a>>; 6] {
    fn register_value(&self, register: u8) -> u8 {
        match register {
            0x01 => {
                let mut value = 0u8;
                for (i, config) in self.iter().enumerate() {
                    if let Some(config) = config {
                        if config.auto_acknowledge {
                            value |= 2u8.pow(i as u32);
                        }
                    }
                }
                return value;
            },
            0x02 => {
                let mut value = 0u8;
                for (i, config) in self.iter().enumerate() {
                    if let Some(config) = config {
                        if config.enabled {
                            value |= 2u8.pow(i as u32);
                        }
                    }
                }
                return value;
            },
            0x1C => {
                let mut value = 0u8;
                for (i, config) in self.iter().enumerate() {
                    if let Some(config) = config {
                        if config.dynamic_payload {
                            value |= 2u8.pow(i as u32);
                        }
                    }
                }
                return value;
            }
            _ => 0,
        }
    }
}