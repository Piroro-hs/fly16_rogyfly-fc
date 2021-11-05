#![no_std]
#![no_main]

mod print;

use cortex_m::asm;
use cortex_m_rt::entry;
#[cfg(not(debug_assertions))]
use panic_halt as _;
use stm32f3xx_hal::{self as hal, pac, prelude::*};

use print::{print, println};

#[entry]
fn main() -> ! {
    asm::delay(8_000 * 5); // Wait 5 ms for clock generator startup

    let dp = pac::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(16.MHz())
        .bypass_hse()
        .sysclk(72.MHz())
        .hclk(72.MHz())
        .pclk1(36.MHz())
        .pclk2(72.MHz())
        .freeze(&mut flash.acr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);

    let tx = gpiob
        .pb3
        .into_af7_push_pull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let rx = gpioa
        .pa15
        .into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    let serial = hal::serial::Serial::new(dp.USART2, (tx, rx), 1_000_000.Bd(), clocks, &mut rcc.apb1);
    let (tx, _) = serial.split();

    print::init(tx);

    loop {
        println!("Hello");
        asm::delay(72_000_000);
    }
}

#[cfg(debug_assertions)]
#[panic_handler]
pub fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("{}", info);
    loop {}
}
