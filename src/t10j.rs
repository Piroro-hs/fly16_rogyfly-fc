use embedded_hal::blocking::delay::DelayMs;

use crate::sbus::{Sbus, SbusPacket};

pub struct T10jState<'a> {
    prev: &'a SbusPacket,
    cur: &'a SbusPacket,
}

impl T10jState<'_> {
    const SBUS_MAX: u16 = 2048;

    pub fn raw(&self) -> &SbusPacket {
        self.cur
    }

    pub fn value(&self, ch: usize) -> u16 {
        self.cur.ch[ch - 1]
    }

    pub fn button(&self, ch: usize) -> (bool, bool) {
        let (position, changed) = self.switch(ch, 2);
        (position == 1, changed)
    }

    pub fn switch(&self, ch: usize, position: u16) -> (u16, bool) {
        let threshold = Self::SBUS_MAX / position;
        let prev = self.prev.ch[ch - 1] / threshold;
        let cur = self.cur.ch[ch - 1] / threshold;
        (cur, !(prev == cur))
    }

    pub fn error(&self) -> bool {
        self.cur.frame_lost | self.cur.failsafe
    }
}

pub struct T10j {
    sbus: Sbus,
    prev: SbusPacket,
    cur: SbusPacket,
    trim: [u16; 4],
}

impl T10j {
    pub fn new(mut sbus: Sbus, delay: &mut impl DelayMs<u16>) -> Option<Self> {
        const TIMEOUT_MS: u16 = 1000;
        const SBUS_INTERVAL_MS: u16 = 14;

        let mut cnt = 0;
        let mut latest;
        while {
            cnt += 1;
            delay.delay_ms(SBUS_INTERVAL_MS);
            latest = sbus.latest();
            (cnt < TIMEOUT_MS / SBUS_INTERVAL_MS) & latest.is_none()
        } {}
        let prev = latest?;
        while {
            cnt += 1;
            delay.delay_ms(SBUS_INTERVAL_MS);
            latest = sbus.latest();
            (cnt <= TIMEOUT_MS / SBUS_INTERVAL_MS) & latest.is_none()
        } {}
        let cur = latest?;
        let trim = (&cur.ch[0..4]).try_into().unwrap();
        Some(Self { sbus, prev, cur, trim })
    }

    pub fn update(&mut self) {
        self.prev = self.cur.clone();
        if let Some(latest) = self.sbus.latest() {
            self.cur = latest;
        }
    }

    pub fn state(&self) -> T10jState {
        T10jState { prev: &self.prev, cur: &self.cur }
    }

    pub fn trim(&self, ch: usize) -> u16 {
        self.trim[ch - 1]
    }
}
