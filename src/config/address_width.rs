//! The address width for all pipes in the nRF24L01 setup

use super::RegisterValue;
use crate::register::Register;

use defmt::Format;

/// The address width of the rx/tx address field
#[derive(Debug, Clone, Copy, Format)]
pub enum AddressWidth {
    // 3 byte address width
    A3Bytes = 1,
    // 4 byte address width
    A4Bytes = 2,
    // 5 byte address width
    A5Bytes = 3,
}

impl AddressWidth {
    /// Get the u8 value of the address width
    pub const fn as_u8(&self) -> u8 {
        match self {
            Self::A3Bytes => 3u8,
            Self::A4Bytes => 4u8,
            Self::A5Bytes => 5u8,
        }
    }
}

impl RegisterValue for AddressWidth {
    fn register_value(&self, register: Register) -> Option<u8> {
        if register == Register::SetupAddressWidths {
            Some(self.as_u8())
        } else {
            None
        }
    }
}