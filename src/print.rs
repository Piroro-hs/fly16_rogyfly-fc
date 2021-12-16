use core::cell::Cell;
use core::fmt::{Arguments, Write};

use cortex_m::interrupt::{free, Mutex};
use stm32f3xx_hal::{
    gpio::{PushPull, AF7, PA2},
    pac::USART2,
};

type Tx = stm32f3xx_hal::serial::Tx<USART2, PA2<AF7<PushPull>>>;

static TX: Mutex<Cell<Option<Tx>>> = Mutex::new(Cell::new(None));

pub fn init(tx: Tx) {
    free(|cs| {
        TX.borrow(cs).set(Some(tx));
    });
}

#[doc(hidden)]
pub fn _print(args: Arguments) {
    free(|cs| {
        if let Some(mut tx) = TX.borrow(cs).take() {
            let _ = tx.write_fmt(args);
            TX.borrow(cs).set(Some(tx));
        }
    })
}

macro_rules! print {
    ($($arg:tt)*) => {
        $crate::print::_print(format_args!($($arg)*))
    };
}
pub(crate) use print;

macro_rules! println {
    () => {
        $crate::print::print!("\n")
    };
    ($($arg:tt)*) => {
        $crate::print::print!("{}\n", format_args!($($arg)*))
    };
}
pub(crate) use println;
