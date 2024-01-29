//! Power Amplifier level for the nRF24l01

use crate::register::Register;

use super::RegisterValue;

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
    PAMax = 3,
    // -6dBm
    PAHigh = 2,
    // -12dBm
    PALow = 1,
    // Minimum Power Amplifier Level (-18dBm)
    PAMin = 0,
}

impl RegisterValue for PowerAmplifier {
    fn register_mask(&self, register: Register) -> u8 {
        if register == Register::RfSetup {
            !(0b0000_0110)
        } else {
            0
        }
    }

    fn register_value(&self, register: Register) -> u8 {
        if register == Register::RfSetup {
            (*self as u8) << 1
        } else {
            0
        }
    }
}