//! The data rate of the module

use super::RegisterValue;
use crate::register::Register;

use defmt::Format;

/// The datarate of transmission
/// 
/// Using lower air rates gives better receiver sensitivity.
/// 
/// Using higher air rates gives lower current consumption and reduced probability
/// of on-air collisions.
#[derive(Debug, Clone, Copy, Format)]
pub enum DataRate {
    // 250 kbps
    R250kb,
    // 1 Mbps
    R1Mb,
    // 2Mbps
    R2Mb,
}

impl RegisterValue for DataRate {
    fn register_value(&self, register: Register) -> Option<u8> {
        if register == Register::RfSetup {
            match self {
                Self::R250kb => Some(1 << 5),
                Self::R1Mb => Some(0),
                Self::R2Mb => Some(1 << 3),
            }
        } else {
            return None;
        }
    }
}