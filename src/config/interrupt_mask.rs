//! The Interrupt Mask for the nRF24L01 radio

use crate::register::Register;

use super::RegisterValue;

use defmt::Format;

/// The interrupt mask for the NRF24l01 Module
#[derive(Debug, Clone, Copy, Format)]
pub struct InterruptMask {
    pub mask_rx: bool,
    pub mask_tx: bool,
    pub mask_retransmits: bool,
}

impl InterruptMask {
    // Create a new interrupt mask with given parameters
    pub const fn new(mask_rx: bool, mask_tx: bool, mask_retransmits: bool) -> Self {
        Self {
            mask_rx,
            mask_tx,
            mask_retransmits,
        }
    }

    // Create an interrupt mask that masks everything except rx
    pub const fn rx() -> Self {
        Self {
            mask_rx: false,
            mask_tx: true,
            mask_retransmits: true,
        }
    }

    // Create an interrupt mask that masks everything except tx
    pub const fn tx() -> Self {
        Self {
            mask_rx: true,
            mask_tx: false,
            mask_retransmits: true,
        }
    }

    // Create an interrupt mask that masks everything except full
    pub const fn retransmits() -> Self {
        Self {
            mask_rx: true,
            mask_tx: true,
            mask_retransmits: false,
        }
    }

    // Create an interrupt mask with no masks
    pub const fn none() -> Self {
        Self {
            mask_rx: false,
            mask_tx: false,
            mask_retransmits: false,
        }
    }
}

impl RegisterValue for InterruptMask {
    fn register_mask(&self, register: Register) -> u8 {
        if register == Register::Config {
            !(0b0111_0000)
        } else {
            0
        }
    }

    fn register_value(&self, register: Register) -> u8 {
        if register == Register::Config {
            let mut value = 0u8;
        
            if self.mask_rx {
                value |= 0b0100_0000;
            }

            if self.mask_tx {
                value |= 0b0010_0000;
            }

            if self.mask_retransmits {
                value |= 0b0001_0000;
            }

            value
        } else {
            0
        }
    }
}