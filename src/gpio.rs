//! General purpose I/O.

use imxrt_iomuxc::gpio::Pin as GpioPin;

/// An output GPIO.
pub struct Output<P: GpioPin<PORT>, const PORT: u8> {
    pin: P,
}

unsafe impl<P: GpioPin<PORT>, const PORT: u8> Send for Output<P, PORT> {}

impl<P: GpioPin<PORT>, const PORT: u8> Drop for Output<P, PORT> {
    fn drop(&mut self) {
        // Reset pin.
        Self::GPIO_INSTANCE
            .pddr()
            .modify(|w| w.set_pdd(Self::IO_NUM, false));
    }
}

impl<P: GpioPin<PORT>, const PORT: u8> Output<P, PORT> {
    // The GPIO instance only has a set of specific values, enforce it for compile errors.
    const GPIO_INSTANCE: imxrt118x_pac::gpio::Gpio = {
        match PORT {
            1 => imxrt118x_pac::RGPIO1,
            2 => imxrt118x_pac::RGPIO2,
            3 => imxrt118x_pac::RGPIO3,
            4 => imxrt118x_pac::RGPIO4,
            5 => imxrt118x_pac::RGPIO5,
            6 => imxrt118x_pac::RGPIO6,
            _ => panic!("Not a valid GPIO instance number"),
        }
    };

    // Extract the IO number (0-31) for GPIO port N from the trait.
    const IO_NUM: usize = <P as GpioPin<PORT>>::OFFSET as usize;

    /// Create a new output pin.
    #[inline(always)]
    pub fn new(pin: P) -> Self {
        Self::GPIO_INSTANCE
            .pddr()
            .modify(|w| w.set_pdd(Self::IO_NUM, true));
        Self { pin }
    }

    /// Set the GPIO high.
    #[inline(always)]
    pub fn set(&self) {
        Self::GPIO_INSTANCE
            .psor()
            .write(|w| w.set_ptso(Self::IO_NUM as _, true));
    }

    /// Set the GPIO low.
    #[inline(always)]
    pub fn clear(&self) {
        Self::GPIO_INSTANCE
            .pcor()
            .write(|w| w.set_ptco(Self::IO_NUM as _, true));
    }

    /// Alternate the GPIO pin output.
    ///
    /// `toggle` is implemented in hardware, so it will be more efficient
    /// than implementing in software.
    #[inline(always)]
    pub fn toggle(&self) {
        Self::GPIO_INSTANCE
            .ptor()
            .write(|w| w.set_ptto(Self::IO_NUM as _, true));
    }

    /// Returns `true` if the GPIO is set.
    #[inline(always)]
    pub fn is_set(&self) -> bool {
        Self::GPIO_INSTANCE.pdor().read().pdo(Self::IO_NUM)
    }

    /// Returns `true` if the value of the pad is high.
    ///
    /// Can differ from [`is_set()`](Self::is_set), especially in an open drain config.
    #[inline(always)]
    pub fn is_pad_high(&self) -> bool {
        Self::GPIO_INSTANCE.pdir().read().pdi(Self::IO_NUM)
    }

    /// Release the underlying pin object.
    #[inline(always)]
    pub fn release(self) -> P {
        // Dismantle the Drop.
        let inner = unsafe { core::ptr::read(&self.pin) };
        core::mem::forget(self);
        inner
    }
}

/// An input GPIO.
pub struct Input<P, const PORT: u8> {
    pin: P,
}

// Safety: see impl Send for Output.
unsafe impl<P: Send, const PORT: u8> Send for Input<P, PORT> {}

// /// Input interrupt triggers.
// #[cfg_attr(feature = "defmt", derive(defmt::Format))]
// #[derive(Debug, Clone, Copy, PartialEq, Eq)]
// pub enum Trigger {
//     /// Trigger is disabled.
//     Disabled,
//     /// Interrupt when GPIO is low.
//     Low,
//     /// Interrupt when GPIO is high.
//     High,
//     /// Interrupt after GPIO rising edge.
//     RisingEdge,
//     /// Interrupt after GPIO falling edge.
//     FallingEdge,
//     /// Interrupt after either a rising or falling edge.
//     EitherEdge,
// }

impl<P: GpioPin<PORT>, const PORT: u8> Input<P, PORT> {
    // The GPIO instance only has a set of specific values, enforce it for compile errors.
    const GPIO_INSTANCE: imxrt118x_pac::gpio::Gpio = {
        match PORT {
            1 => imxrt118x_pac::RGPIO1,
            2 => imxrt118x_pac::RGPIO2,
            3 => imxrt118x_pac::RGPIO3,
            4 => imxrt118x_pac::RGPIO4,
            5 => imxrt118x_pac::RGPIO5,
            6 => imxrt118x_pac::RGPIO6,
            _ => panic!("Not a valid GPIO instance number"),
        }
    };

    // Extract the IO number (0-31) for GPIO port N from the trait.
    const IO_NUM: usize = <P as GpioPin<PORT>>::OFFSET as usize;

    /// Create a new input pin.
    #[inline(always)]
    pub fn new(pin: P) -> Self {
        Self::GPIO_INSTANCE
            .pddr()
            .modify(|w| w.set_pdd(Self::IO_NUM, false));
        Self { pin }
    }

    /// Returns `true` if the GPIO is set high.
    #[inline(always)]
    pub fn is_set(&self) -> bool {
        Self::GPIO_INSTANCE.pdir().read().pdi(Self::IO_NUM)
    }

    /// Release the underlying pin object.
    #[inline(always)]
    pub fn release(self) -> P {
        self.pin
    }

    // TODO: Add wakers and implement `embedded_hal_async::digital::Wait`.
    // /// Enable or disable GPIO input interrupts.
    // ///
    // /// Specify `None` to disable interrupts. Or, provide a trigger
    // /// to configure the interrupt.
    // #[inline(always)]
    // pub fn set_interrupt(&mut self, trigger: Trigger) {
    //     Self::GPIO_INSTANCE.icr(Self::IO_NUM).modify(|w| {
    //         use imxrt118x_pac::gpio::vals::Irqc::*;
    //
    //         w.set_irqc(match trigger {
    //             Trigger::Disabled => IRQC0,
    //             Trigger::Low => IRQC8,
    //             Trigger::High => IRQC12,
    //             Trigger::RisingEdge => IRQC9,
    //             Trigger::FallingEdge => IRQC10,
    //             Trigger::EitherEdge => IRQC11,
    //         })
    //     })
    // }
    //
    // /// Returns `true` if the GPIO interrupt has triggered.
    // #[inline(always)]
    // pub fn is_triggered(&self) -> bool {
    //     Self::GPIO_INSTANCE.icr(Self::IO_NUM).read().isf()
    // }
    //
    // /// Clear the interrupt triggered flag.
    // #[inline(always)]
    // pub fn clear_triggered(&mut self) {
    //     Self::GPIO_INSTANCE
    //         .icr(Self::IO_NUM)
    //         .write(|w| w.set_isf(true))
    // }
}

impl<P: GpioPin<PORT>, const PORT: u8> embedded_hal::digital::ErrorType for Output<P, PORT> {
    type Error = core::convert::Infallible;
}

impl<P: GpioPin<PORT>, const PORT: u8> embedded_hal::digital::OutputPin for Output<P, PORT> {
    #[inline(always)]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Output::set(self);
        Ok(())
    }

    #[inline(always)]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        Output::clear(self);
        Ok(())
    }
}

impl<P: GpioPin<PORT>, const PORT: u8> embedded_hal::digital::StatefulOutputPin
    for Output<P, PORT>
{
    #[inline(always)]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Output::is_set(self))
    }

    #[inline(always)]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!Output::is_set(self))
    }

    #[inline(always)]
    fn toggle(&mut self) -> Result<(), Self::Error> {
        Output::toggle(self);
        Ok(())
    }
}

// For open drain or simply reading back the actual state
// of the pin.
impl<P: GpioPin<PORT>, const PORT: u8> embedded_hal::digital::InputPin for Output<P, PORT> {
    #[inline(always)]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Output::is_pad_high(self))
    }

    #[inline(always)]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!Output::is_pad_high(self))
    }
}

impl<P: GpioPin<PORT>, const PORT: u8> embedded_hal::digital::ErrorType for Input<P, PORT> {
    type Error = core::convert::Infallible;
}

impl<P: GpioPin<PORT>, const PORT: u8> embedded_hal::digital::InputPin for Input<P, PORT> {
    #[inline(always)]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(Input::is_set(self))
    }

    #[inline(always)]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(!Input::is_set(self))
    }
}
