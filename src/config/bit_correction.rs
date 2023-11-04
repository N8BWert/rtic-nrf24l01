//! The Bit Correction Level of the nRF24L01 module

use super::RegisterValue;

use defmt::Format;

/// The mandatory error detection mechanism in the packet
#[derive(Debug, Clone, Copy, Format)]
pub enum BitCorrection {
    // 1 byte CRC bit correction
    CRC1,
    // 2 byte CRC bit correction
    CRC2,
}

impl RegisterValue for BitCorrection {
    fn register_value(&self, address: u8) -> u8 {
        if address == 0 {
            match self {
                Self::CRC1 => 0b0000_1000,
                Self::CRC2 => 0b0000_1100,
            }
        } else {
            return 0;
        }
    }
}