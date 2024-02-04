//! 
//! CRC Value on the nRF24L01+ Module
//! 

use crate::register::Register;

use super::RegisterValue;

use defmt::Format;

#[derive(Debug, Clone, Copy, Format)]
pub enum CRC {
    // No CRC
    CRC0,
    // 1 Byte CRC Check
    CRC1,
    // 2 Byte CRC Check
    CRC2,
}

impl RegisterValue for CRC {
    fn register_mask(&self, register: Register) -> u8 {
        if register == Register::Config {
            !(0b0000_1100)
        } else {
            0
        }
    }

    fn register_value(&self, register: Register) -> u8 {
        if register == Register::Config {
            match self {
                Self::CRC0 => 0x00,
                Self::CRC1 => 0b0000_1000,
                Self::CRC2 => 0b0000_1100,
            }
        } else {
            0
        }
    }
}