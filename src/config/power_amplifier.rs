//! Power Amplifier level for the nRF24l01

use super::RegisterValue;
use crate::register::Register;

use defmt::Format;

/// The power amplifier configuration
/// 
/// Output Power | Current Consumption
/// ----------------------------------
/// 0dBm         | 11.3mA
/// ----------------------------------
/// -6dBm        | 9.0mA
/// ----------------------------------
/// -12dBm       | 7.5mA
/// ----------------------------------
/// -18dBm       | 7.0mA
/// ----------------------------------
#[derive(Debug, Clone, Copy, Format)]
pub enum PowerAmplifier {
    // Maximum Power Amplifier Level (0dBm)
    PAMax,
    // -6dBm
    PAHigh,
    // -12dBm
    PAMedium,
    // Minimum Power Amplifier Level (-18dBm)
    PALow,
}

impl RegisterValue for PowerAmplifier {
    fn register_value(&self, register: Register) -> Option<u8> {
        if register == Register::RfSetup {
            match self {
                Self::PAMax => Some(3 << 1),
                Self::PAHigh => Some(2 << 1),
                Self::PAMedium => Some(1 << 1),
                Self::PALow => Some(0),
            }
        } else {
            None
        }
    }
}