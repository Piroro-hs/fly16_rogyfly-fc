use cortex_m::{
    asm,
    peripheral::{syst::SystClkSource, SYST},
};
use cortex_m_rt::exception;
use stm32f3xx_hal::rcc::Clocks;

#[derive(Debug)]
pub struct SysClock(());

impl SysClock {
    pub fn new(mut syst: SYST, clocks: Clocks) -> Self {
        syst.set_clock_source(SystClkSource::External); // HCLK / 8
        syst.set_reload(clocks.hclk().0 / 8 / 100 - 1); // 100 Hz
        syst.clear_current();
        syst.enable_interrupt();
        syst.enable_counter();
        Self(())
    }

    #[inline]
    pub fn wait(&self) {
        asm::wfi();
    }
}

#[exception]
fn SysTick() {}
