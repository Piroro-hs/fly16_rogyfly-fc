#![no_std]
#![no_main]

mod clock;
mod delay;
mod print;
mod sbus;

use bno055::{Bno055, BNO055OperationMode};
use cortex_m::asm;
use cortex_m_rt::entry;
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

    let pin = gpioa.pa10.into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    let dma1 = dp.DMA1.split(&mut rcc.ahb);
    let mut sbus = sbus::Sbus::new(dp.USART1, pin, dma1.ch5, clocks, &mut rcc.apb2);

    // Use timers on APB1 to avoid stm32f3xx-hal clock configure bug
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

    let mut imu = Bno055::new(i2c).with_alternative_address();

    imu.init(&mut delay).unwrap();
    imu.set_external_crystal(true, &mut delay).unwrap();
    // imu.set_axis_remap(AxisRemap::builder().swap_x_with(Axis::AXIS_AS_Z).build().unwrap()).unwrap();
    imu.set_mode(BNO055OperationMode::IMU, &mut delay).unwrap();
    // while !imu.is_fully_calibrated().unwrap() {
    //     sys_clock.wait();
    // }

    loop {
        println!("Hello");
        if let Some(data) = sbus.latest() {
            const SERVO_PWM_OFFSET: u16 = (1.52 / 0.000625 - 1024.0) as u16;
            const AILERON_DIFF_RATIO: f32 = 0.6;

            println!("S.BUS: {:?}", data);
            let a = (data.ch[0] as i16 - 1024) as f32;
            aileron_l.set_duty((1024 + (a * if a > 0.0 { 1.0 } else { AILERON_DIFF_RATIO }) as i16) as u16 + SERVO_PWM_OFFSET);
            aileron_r.set_duty((1024 + (a * if a < 0.0 { 1.0 } else { AILERON_DIFF_RATIO }) as i16) as u16 + SERVO_PWM_OFFSET);
            elevator.set_duty(data.ch[1] + SERVO_PWM_OFFSET);
            throttle.set_duty((data.ch[2] + SERVO_PWM_OFFSET) as u32);
            rudder.set_duty(data.ch[3] + SERVO_PWM_OFFSET);
        }
        if let Ok(quat) = imu.quaternion() {
            println!("Quaternion: {:?}", quat);
        }
        if let Ok(acc) = imu.linear_acceleration() {
            println!("Acceleration: {:?}", acc);
        }

        sys_clock.wait();
    }
}

#[cfg(debug_assertions)]
#[panic_handler]
pub fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("{}", info);
    loop {}
}
