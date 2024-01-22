//! The Bit Correction Level of the nRF24L01 module

use super::RegisterValue;
use crate::register::Register;

use defmt::Format;

/// The mandatory error detection mechanism in the packet
#[derive(Debug, Clone, Copy, Format)]
pub enum CRC {
    // No CRC 
    CRCNone,
    // 1 byte CRC bit correction
    CRC1,
    // 2 byte CRC bit correction
    CRC2,
}

impl RegisterValue for CRC {
    fn register_value(&self, register: Register) -> Option<u8> {
        if register == Register::Config {
            match self {
                Self::CRCNone => Some(0),
                Self::CRC1 => Some(2 << 1),
                Self::CRC2 => Some(3 << 1),
            }
        } else {
            None
        }
    }
}