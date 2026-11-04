//! Signal processing ahead of the decoder.

pub mod resampler;
pub mod spectrum;

pub use resampler::ChannelFilter;
pub use spectrum::Spectrum;
