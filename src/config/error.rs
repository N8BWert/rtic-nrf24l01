//! Error for creating a given configuration

use defmt::Format;

/// Configuration Error (only used for ensuring a valid rf channel)
#[derive(Debug, Clone, Copy, Format)]
pub enum ConfigurationError {
    // The rf channel provided to the configuration is not within the valid range [0,124]
    InvalidRfChannel,
    // The length of address provided in the data pipe config is not the same as what was provided in
    // the global config
    InvalidAddressLength,
    // The transmit delay provided was too large (greater than 0xf)
    InvalidTransmitDelay,
    // The transmit count provided was too large (greater than 0xf)
    InvalidTransmitCount,
    // The tx address is not the correct length
    InvalidTxAddressLength,
    // The Receive addresses do not all have the same first x digits
    InvalidRxAddresses,
    // The first pipe config is missing,
    MissingFirstPipeConfig,
}