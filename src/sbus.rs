use stm32f3xx_hal::{
    dma::{dma1, Channel, Increment, Transfer},
    gpio::{self, PushPull, AF7},
    pac::{DMA1, RCC, USART1},
    rcc::{Clocks, APB2},
    serial::Rx,
};

const BUF_LEN: usize = u8::MAX as usize + 1;

static mut BUFFER: [u8; BUF_LEN] = [0; BUF_LEN];

#[derive(Clone, Debug)]
pub struct SbusPacket {
    pub ch: [u16; 16],
    pub ch17: bool,
    pub ch18: bool,
    pub frame_lost: bool,
    pub failsafe: bool,
}

type Pin = gpio::PA10<AF7<PushPull>>;

pub struct Sbus {
    _transfer: Transfer<&'static mut [u8; BUF_LEN], dma1::C5, Rx<USART1, Pin>>,
    cur: u8,
}

impl Sbus {
    const BAUD: u32 = 100_000;
    const PACKET_LEN: u8 = 25;
    const HEADER: u8 = 0x0F;
    const FOOTER: u8 = 0x00;
    // const FOOTER: u8 = 0x04; // S.BUS2 0x04~0x34
    // const FOOTER_MSK: u8 = 0x0F;

    pub fn new(
        usart: USART1,
        _pin: Pin,
        mut channel: dma1::C5,
        clocks: Clocks,
        _apb: &mut APB2,
    ) -> Self {
        // apb.enr().modify(|_, w| w.usart1en().set_bit());
        // apb.rstr().modify(|_, w| w.usart1rst().set_bit());
        // apb.rstr().modify(|_, w| w.usart1rst().clear_bit());
        unsafe { &(*RCC::ptr()) }.apb2enr.modify(|_, w| w.usart1en().set_bit());
        unsafe { &(*RCC::ptr()) }.apb2rstr.modify(|_, w| w.usart1rst().set_bit());
        unsafe { &(*RCC::ptr()) }.apb2rstr.modify(|_, w| w.usart1rst().clear_bit());

        usart.cr1.modify(|_, w| {
            w.m().bit9();
            w.pce().enabled();
            w.ps().even()
        });
        usart.cr2.modify(|_, w| {
            w.rxinv().inverted()
            // w.stop().stop2()
        });
        usart.cr3.modify(|_, w| w.ovrdis().disabled());
        usart.brr.write(|w| w.brr().bits((clocks.pclk1().0 / Self::BAUD) as u16));

        usart.cr1.modify(|_, w| {
            w.re().enabled();
            w.ue().enabled()
        });

        let transfer = unsafe {
            let address = &(*USART1::ptr()).rdr as *const _ as u32;
            channel.set_peripheral_address(address, Increment::Disable);
            (*DMA1::ptr()).ch5.cr.modify(|_, w| w.circ().enabled());
            let usart_rx: Rx<USART1, Pin> = core::mem::zeroed();
            Transfer::start_write(&mut BUFFER, channel, usart_rx)
        };

        Self {
            _transfer: transfer,
            cur: 0,
        }
    }

    pub fn latest(&mut self) -> Option<SbusPacket> {
        let mut latest_cur = None;
        let next_cur =
            (BUF_LEN as u16 - unsafe { &(*DMA1::ptr()) }.ch5.ndtr.read().ndt().bits()) as u8;
        while self.cur != next_cur {
            match Self::read_buffer(self.cur) {
                Self::HEADER if next_cur.wrapping_sub(self.cur) < Self::PACKET_LEN => break,
                Self::HEADER if Self::read_buffer(self.cur.wrapping_add(Self::PACKET_LEN - 1)) == Self::FOOTER => {
                    latest_cur = Some(self.cur);
                    self.cur = self.cur.wrapping_add(Self::PACKET_LEN);
                }
                _ => self.cur = self.cur.wrapping_add(1),
            }
        }

        latest_cur.map(|cur| {
            let mut data = [0_u8; Self::PACKET_LEN as usize - 2];
            for (i, elem) in data.iter_mut().enumerate() {
                *elem = Self::read_buffer(cur.wrapping_add(i as u8 + 1));
            }

            let mut ch = [0_u16; 16];
            for (i, elem) in ch.iter_mut().enumerate() {
                let idx = i + 3 * i / 8;
                let shift = 3 * i % 8;
                *elem = ((data[idx] as u16) >> shift
                    | (data[idx + 1] as u16) << (8 - shift)
                    | if 16 - shift < 11 { (data[idx + 2] as u16) << (16 - shift) } else { 0 })
                    & 0x07FF;
            }

            SbusPacket {
                ch,
                ch17: data[22] & 0x01 != 0,
                ch18: data[22] & 0x02 != 0,
                frame_lost: data[22] & 0x04 != 0,
                failsafe: data[22] & 0x08 != 0,
            }
        })
    }

    fn read_buffer(cur: u8) -> u8 {
        unsafe { BUFFER.as_ptr().add(cur as usize).read_volatile() }
    }
}
