use embedded_hal::{
    blocking::delay::DelayMs,
    blocking::i2c::{Write, WriteRead},
};

const ADDR: u8 = 0x56;

pub struct Barometer<I> {
    i2c: I,
    a0: f64,
    a1: f64,
    a2: f64,
    b00: f64,
    bt1: f64,
    bt2: f64,
    bp1: f64,
    b11: f64,
    bp2: f64,
    b12: f64,
    b21: f64,
    bp3: f64,
}

impl<I, E> Barometer<I>
where
    I: WriteRead<Error = E> + Write<Error = E>,
{
    pub fn new(mut i2c: I, delay: &mut impl DelayMs<u16>) -> Result<Self, E> {
        i2c.write(ADDR, &[0xF5, 0x00])?; // t_standby 1 ms
        // i2c.write(ADDR, &[0xF1, 0b0000_0010])?; // IIR filter N = 4
        // i2c.write(ADDR, &[0xF1, 0b0000_0101])?; // IIR filter N = 32
        i2c.write(ADDR, &[0xF1, 0b0000_0000])?; // IIR filter OFF

        let mut coe = [0_u8; 25];
        i2c.write_read(ADDR, &[0xA0], &mut coe)?;
        let a0 = (i32::from_be_bytes([coe[18], coe[19], coe[24] << 4, 0]) >> 12) as f64 / 16.0; // Q15.4
        let a1 =
            -6.3E-03 + 4.3E-04 * i16::from_be_bytes(coe[20..22].try_into().unwrap()) as f64 / 32767.0;
        let a2 =
            -1.9E-11 + 1.2E-10 * i16::from_be_bytes(coe[22..24].try_into().unwrap()) as f64 / 32767.0;
        // let a1 = -6.3E-3 + 4.3E-4 * i16::from_be_bytes([coe[20], coe[21]]) as f64 / 32767.0;
        // let a2 = -1.9E-11 + 1.2E-10 * i16::from_be_bytes([coe[22], coe[23]]) as f64 / 32767.0;
        let b00 = (i32::from_be_bytes([coe[0], coe[1], coe[24] & 0xF0, 0]) >> 12) as f64 / 16.0; // Q15.4
        let bt1 =
            1.0E-01 + 9.1E-02 * i16::from_be_bytes(coe[2..4].try_into().unwrap()) as f64 / 32767.0;
        let bt2 =
            1.2E-08 + 1.2E-06 * i16::from_be_bytes(coe[4..6].try_into().unwrap()) as f64 / 32767.0;
        let bp1 =
            3.3E-02 + 1.9E-02 * i16::from_be_bytes(coe[6..8].try_into().unwrap()) as f64 / 32767.0;
        let b11 =
            2.1E-07 + 1.4E-07 * i16::from_be_bytes(coe[8..10].try_into().unwrap()) as f64 / 32767.0;
        let bp2 =
            -6.3E-10 + 3.5E-10 * i16::from_be_bytes(coe[10..12].try_into().unwrap()) as f64 / 32767.0;
        let b12 =
            2.9E-13 + 7.6E-13 * i16::from_be_bytes(coe[12..14].try_into().unwrap()) as f64 / 32767.0;
        let b21 =
            2.1E-15 + 1.2E-14 * i16::from_be_bytes(coe[14..16].try_into().unwrap()) as f64 / 32767.0;
        let bp3 =
            1.3E-16 + 7.9E-17 * i16::from_be_bytes(coe[16..18].try_into().unwrap()) as f64 / 32767.0;
        delay.delay_ms(1); // Wait 1 ms

        i2c.write(ADDR, &[0xF4, 0b0010_1111])?; // temp 1, pressure 4, Normal mode

        Ok(Self {
            i2c,
            a0,
            a1,
            a2,
            b00,
            bt1,
            bt2,
            bp1,
            b11,
            bp2,
            b12,
            b21,
            bp3,
        })
    }

    pub fn pressure(&mut self) -> Result<f64, E> {
        let mut res = [0_u8; 6];
        self.i2c.write_read(ADDR, &[0xF7], &mut res)?;
        let dt = (i32::from_be_bytes([0, res[3], res[4], res[5]]) - 0x80_0000) as f64;
        let tr = self.a0 + self.a1 * dt + self.a2 * dt * dt;
        let dp = (i32::from_be_bytes([0, res[0], res[1], res[2]]) - 0x80_0000) as f64;
        let pr = self.b00
            + self.bt1 * tr
            + self.bp1 * dp
            + self.b11 * dp * tr
            + self.bt2 * tr * tr
            + self.bp2 * dp * dp
            + self.b12 * dp * tr * tr
            + self.b21 * dp * dp * tr
            + self.bp3 * dp * dp * dp;
        Ok(pr)
    }
}
