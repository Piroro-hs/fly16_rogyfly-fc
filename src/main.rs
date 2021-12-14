#![no_std]
#![no_main]

mod barometer;
mod clock;
mod delay;
mod print;
mod sbus;
mod t10j;

use bno055::{Bno055, BNO055OperationMode};
use cortex_m::asm;
use cortex_m_rt::entry;
use lps22hb::{fifo::FIFOConfig, interface::{I2cInterface, i2c::I2cAddress}, LPS22HB};
#[cfg(not(debug_assertions))]
use panic_halt as _;
use stm32f3xx_hal::{self as hal, pac, prelude::*};

use print::{print, println};

#[entry]
fn main() -> ! {
    asm::delay(8_000 * 5); // Wait 5 ms for clock generator startup

    let cp = pac::CorePeripherals::take().unwrap();
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

    let sys_clock = clock::SysClock::new(cp.SYST, clocks);

    let mut delay = delay::Delay::new(cp.DWT, cp.DCB, clocks);

    delay.delay_ms(1000);

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

    println!("Hello");

    let pin = gpioa.pa10.into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    let dma1 = dp.DMA1.split(&mut rcc.ahb);
    let sbus = sbus::Sbus::new(dp.USART1, pin, dma1.ch5, clocks, &mut rcc.apb2);

    let mut t10j = t10j::T10j::new(sbus, &mut delay);

    // Use timers on APB1 to avoid stm32f3xx-hal clock configuration issue
    let (.., t2c4) = hal::pwm::tim2(dp.TIM2, 16000, 100.Hz(), &clocks);
    let (t3c1, t3c2, t3c3, t3c4) = hal::pwm::tim3(dp.TIM3, 16000, 100.Hz(), &clocks);
    let mut throttle = t2c4.output_to_pa3(gpioa.pa3.into_af1_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl));
    let mut aileron_l = t3c2.output_to_pa4(gpioa.pa4.into_af2_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl));
    let mut rudder = t3c1.output_to_pa6(gpioa.pa6.into_af2_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl));
    let mut elevator = t3c3.output_to_pb0(gpiob.pb0.into_af2_push_pull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl));
    let mut aileron_r = t3c4.output_to_pb1(gpiob.pb1.into_af2_push_pull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl));

    aileron_l.enable();
    aileron_r.enable();
    elevator.enable();
    throttle.enable();
    rudder.enable();

    let mut scl = gpiob.pb6.into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let mut sda = gpiob.pb7.into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    scl.internal_pull_up(&mut gpiob.pupdr, true);
    sda.internal_pull_up(&mut gpiob.pupdr, true);
    let i2c = hal::i2c::I2c::new(dp.I2C1, (scl, sda), 400_000.Hz(), clocks, &mut rcc.apb1);

    let bus = shared_bus::BusManagerSimple::new(i2c);

    // let mut imu = Bno055::new(i2c).with_alternative_address();
    let mut imu = Bno055::new(bus.acquire_i2c());

    imu.init(&mut delay).unwrap();
    imu.set_external_crystal(true, &mut delay).unwrap();
    // imu.set_axis_remap(AxisRemap::builder().swap_x_with(Axis::AXIS_AS_Z).build().unwrap()).unwrap();
    imu.set_mode(BNO055OperationMode::IMU, &mut delay).unwrap();
    delay.delay_ms(100); // BNO055 returns constant zero without this delay
    imu.set_calibration_profile(BNO055Calibration::from_buf(
        &[0xED, 0xFF, 0x0E, 0x00, 0x1E, 0x00, 0xED, 0xFF, 0x0E, 0x00, 0x1E, 0x00, 0xFE, 0xFF, 0x01, 0x00, 0xFF, 0xFF, 0xE8, 0x03, 0x68, 0x03]
    ), &mut delay).unwrap();
    delay.delay_ms(100);

    let mut barometer_o = barometer::Barometer::new(bus.acquire_i2c(), &mut delay, Some(|| unsafe {
        (*pac::I2C1::ptr()).cr1.modify(|_, w| w.pe().disabled());
        asm::delay(3 * clocks.ppre1() as u32); // Wait at least 3 APB clock cycles
        (*pac::I2C1::ptr()).cr1.modify(|_, w| w.pe().enabled());
    })).unwrap();

    let mut barometer_s = LPS22HB::new(I2cInterface::init(bus.acquire_i2c(), I2cAddress::SA0_VCC));
    barometer_s.software_reset().unwrap();
    barometer_s.set_datarate(lps22hb::ODR::_75Hz).unwrap();
    let fifo_config = FIFOConfig { fifo_mode: lps22hb::FIFO_MODE::Stream, ..Default::default() };
    barometer_s.enable_fifo(false, fifo_config).unwrap();
    barometer_s.lowpass_filter(true, true).unwrap();

    const SAMPLE_NUM: usize = 10;
    let (acc_o, acc_s) = (0..SAMPLE_NUM)
        .map(|_| {
            delay.delay_ms(100);
            (barometer_o.pressure().unwrap(), barometer_s.read_pressure().unwrap() * 100.0)
        })
        .fold(
            ((0.0, f64::NEG_INFINITY, f64::INFINITY), (0.0, f32::NEG_INFINITY, f32::INFINITY)),
            |(acc_o, acc_s), (cur_o, cur_s)| ((acc_o.0 + cur_o, acc_o.1.max(cur_o), acc_o.2.min(cur_o)), (acc_s.0 + cur_s, acc_s.1.max(cur_s), acc_s.2.min(cur_s))),
        );
    let pressure_mean_o = (acc_o.0 - (acc_o.1 + acc_o.2)) / (SAMPLE_NUM - 2) as f64;
    let pressure_mean_s = (acc_s.0 - (acc_s.1 + acc_s.2)) / (SAMPLE_NUM - 2) as f32;

    delay.delay_ms(1000);

    println!("Start");

    let mut cnt = 0;
    loop {
        const SERVO_PWM_OFFSET: u16 = (1.52 / 0.000625 - 1024.0) as u16;
        const AILERON_DIFF_RATIO: f32 = 0.6;

        t10j.update();
        let state = t10j.state();
        println!("S.BUS: {:?}", state.raw());

        let a = (state.value(1) as i16 - 1024) as f32;
        aileron_l.set_duty((1024 + (a * if a > 0.0 { 1.0 } else { AILERON_DIFF_RATIO }) as i16) as u16 + SERVO_PWM_OFFSET);
        aileron_r.set_duty((1024 + (a * if a < 0.0 { 1.0 } else { AILERON_DIFF_RATIO }) as i16) as u16 + SERVO_PWM_OFFSET);
        elevator.set_duty(state.value(2) + SERVO_PWM_OFFSET);
        throttle.set_duty((state.value(3) + SERVO_PWM_OFFSET) as u32);
        rudder.set_duty(state.value(4) + SERVO_PWM_OFFSET);

        if let Ok(quat) = imu.quaternion() {
            println!("Quaternion: {:?}", quat);
        }
        if let Ok(acc) = imu.linear_acceleration() {
            println!("Acceleration: {:?}", acc);
        }
        if if cnt == 0 {
            if let (Ok(pressure_o), Ok(pressure_s)) = (
                barometer_o.pressure(),
                barometer_s.read_pressure(),
            ) {
                let altitude_o = (pressure_o - pressure_mean_o) * -8.344407986; // https://en.wikipedia.org/wiki/Pressure_altitude
                let altitude_s = (pressure_s * 100.0 - pressure_mean_s) * -8.344407986; // https://en.wikipedia.org/wiki/Pressure_altitude
                let altitude = (a_o as f32 + a_s) / 2.0;
                println!("Altitude: {:?}", altitude);
            }
        }
        cnt = (cnt + 1) % 10;

        sys_clock.wait();
    }
}

#[cfg(debug_assertions)]
#[panic_handler]
pub fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("{}", info);
    loop {}
}
