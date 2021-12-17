use core::convert::Infallible;

use embedded_hal::{blocking::delay::DelayMs, digital::v2::OutputPin};

#[derive(Clone, Copy, Debug)]
pub enum Color {
    Red,
    Green,
    Blue,
    Yellow,
    Cyan,
    Magenta,
    White,
}

pub struct Led<R, G, B> {
    red: R,
    green: G,
    blue: B,
    state: Option<Color>,
}

impl<R, G, B> Led<R, G, B>
where
    R: OutputPin<Error = Infallible>,
    G: OutputPin<Error = Infallible>,
    B: OutputPin<Error = Infallible>,
{
    pub fn new(red: R, green: G, blue: B) -> Self {
        Self { red, green, blue, state: None }
    }

    pub fn set(&mut self, state: Option<Color>) {
        match state {
            Some(Color::Red) => {
                self.red.set_high().unwrap();
                self.green.set_low().unwrap();
                self.blue.set_low().unwrap();
            }
            Some(Color::Green) => {
                self.red.set_low().unwrap();
                self.green.set_high().unwrap();
                self.blue.set_low().unwrap();
            }
            Some(Color::Blue) => {
                self.red.set_low().unwrap();
                self.green.set_low().unwrap();
                self.blue.set_high().unwrap();
            }
            Some(Color::Yellow) => {
                self.red.set_high().unwrap();
                self.green.set_high().unwrap();
                self.blue.set_low().unwrap();
            }
            Some(Color::Cyan) => {
                self.red.set_low().unwrap();
                self.green.set_high().unwrap();
                self.blue.set_high().unwrap();
            }
            Some(Color::Magenta) => {
                self.red.set_high().unwrap();
                self.green.set_low().unwrap();
                self.blue.set_high().unwrap();
            }
            Some(Color::White) => {
                self.red.set_high().unwrap();
                self.green.set_high().unwrap();
                self.blue.set_high().unwrap();
            }
            None => {
                self.red.set_low().unwrap();
                self.green.set_low().unwrap();
                self.blue.set_low().unwrap();
            }
        }
        self.state = state;
    }

    pub fn blink(&mut self, color: Color, cnt: usize, delay: &mut impl DelayMs<u16>) {
        const BLINK_MS: u16 = 100;

        let prev = self.state;
        for _ in 0..cnt {
            self.set(None);
            delay.delay_ms(BLINK_MS);
            self.set(Some(color));
            delay.delay_ms(BLINK_MS);
        }
        self.set(None);
        delay.delay_ms(BLINK_MS);
        self.set(prev);
    }
}
