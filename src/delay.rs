use embedded_hal::blocking::delay::DelayMs;
use stm32f3xx_hal::{pac::{DCB, DWT}, rcc::Clocks};

#[derive(Clone, Copy)]
pub struct Delay {
    tick: u32,
}

impl Delay {
    pub fn new(mut dwt: DWT, mut dcb: DCB, clocks: Clocks) -> Self {
        dcb.enable_trace();
        DWT::unlock();
        dwt.enable_cycle_counter();
        Self { tick: clocks.hclk().0 / 1000 }
    }
}

impl DelayMs<u16> for Delay {
    fn delay_ms(&mut self, ms: u16) {
        // FIXME: consider potential overflow and race conditions
        let start = DWT::get_cycle_count();
        let ticks = self.tick * ms as u32;
        while (DWT::get_cycle_count().wrapping_sub(start)) < ticks {}
    }
}
