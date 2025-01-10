#![no_std]

use core::sync::atomic::{AtomicBool, Ordering};

pub use imxrt118x_pac as pac;
use imxrt_iomuxc::imxrt1180::{
    gpio_ad, gpio_aon, gpio_b1, gpio_b2, gpio_emc_b1, gpio_emc_b2, gpio_sd_b1, gpio_sd_b2,
};

mod clocks;
mod dma;
pub mod gpio;
mod lpuart;

/// Initialize the HAL.
///
/// It's guaranteed that this will only be able to run once.
pub fn init(_config: Config) -> Peripherals {
    // Enforce single use of init.
    static TAKEN: AtomicBool = AtomicBool::new(false);
    if Ok(false) != TAKEN.compare_exchange(false, true, Ordering::Relaxed, Ordering::Relaxed) {
        panic!("HAL initialized multiple times");
    }

    // Safety: These are only called once.
    Peripherals {
        gpio_ad: unsafe { gpio_ad::Pads::new() },
        gpio_aon: unsafe { gpio_aon::Pads::new() },
        gpio_b1: unsafe { gpio_b1::Pads::new() },
        gpio_b2: unsafe { gpio_b2::Pads::new() },
        gpio_emc_b1: unsafe { gpio_emc_b1::Pads::new() },
        gpio_emc_b2: unsafe { gpio_emc_b2::Pads::new() },
        gpio_sd_b1: unsafe { gpio_sd_b1::Pads::new() },
        gpio_sd_b2: unsafe { gpio_sd_b2::Pads::new() },
    }
}

/// System configuration.
#[derive(Default, defmt::Format)]
pub struct Config {}

/// All peripherals.
pub struct Peripherals {
    pub gpio_ad: gpio_ad::Pads,
    pub gpio_aon: gpio_aon::Pads,
    pub gpio_b1: gpio_b1::Pads,
    pub gpio_b2: gpio_b2::Pads,
    pub gpio_emc_b1: gpio_emc_b1::Pads,
    pub gpio_emc_b2: gpio_emc_b2::Pads,
    pub gpio_sd_b1: gpio_sd_b1::Pads,
    pub gpio_sd_b2: gpio_sd_b2::Pads,
}
