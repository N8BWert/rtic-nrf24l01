//! The address width for all pipes in the nRF24L01 setup

use crate::register::Register;

use super::RegisterValue;

use defmt::Format;

/// The address width of the rx/tx address field
#[derive(Debug, Clone, Copy, Format, PartialEq, Eq)]
pub enum AddressWidth {
    // 3 byte address width
    A3Bytes = 0x01,
    // 4 byte address width
    A4Bytes = 0x10,
    // 5 byte address width
    A5Bytes = 0x11,
}

impl AddressWidth {
    /// Get the u8 value of the address width
    pub const fn as_usize(&self) -> usize {
        match self {
            Self::A3Bytes => 3,
            Self::A4Bytes => 4,
            Self::A5Bytes => 5,
        }
    }
}

impl RegisterValue for AddressWidth {
    fn register_mask(&self, register: Register) -> u8 {
        if register == Register::SetupAddressWidth {
            !(0b0000_0011)
        } else {
            0
        }
    }

    fn register_value(&self, register: Register) -> u8 {
        if register == Register::SetupAddressWidth {
            *self as u8
        } else {
            0
        }
    }
}