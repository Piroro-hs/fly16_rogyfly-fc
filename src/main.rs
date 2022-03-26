#![no_std]
#![no_main]

mod barometer;
mod clock;
mod delay;
mod led;
mod print;
mod sbus;
mod t10j;
mod util;

use bno055::{BNO055Calibration, BNO055OperationMode, Bno055};
use cortex_m::asm;
use cortex_m_rt::entry;
use glam::{f32::*, EulerRot};
use lps22hb::{fifo::FIFOConfig, interface::{I2cInterface, i2c::I2cAddress}, LPS22HB};
#[cfg(not(debug_assertions))]
use panic_halt as _;
use pid::Pid;
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

    let tx = gpioa.pa2.into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let rx = gpioa.pa3.into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let serial = hal::serial::Serial::new(dp.USART2, (tx, rx), 1_000_000.Bd(), clocks, &mut rcc.apb1);
    let (tx, _) = serial.split();

    print::init(tx);

    println!("Hello");

    let mut ejector = gpioa.pa7.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    let mut ejector_start_cnt = 0;

    let red = gpioa.pa11.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    let green = gpioa.pa12.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    let blue = gpioa.pa8.into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    let mut led = led::Led::new(red, green, blue);

    let pin = gpioa.pa10.into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    let dma1 = dp.DMA1.split(&mut rcc.ahb);
    let sbus = sbus::Sbus::new(dp.USART1, pin, dma1.ch5, clocks, &mut rcc.apb2);

    let mut t10j = t10j::T10j::new(sbus, &mut delay);
    if t10j.is_none() {
        println!("S.BUS signal not found");
    }

    // Use timers on APB1 to avoid stm32f3xx-hal clock configuration issue
    let (t2c1, ..) = hal::pwm::tim2(dp.TIM2, 16000, 100.Hz(), &clocks);
    let (t3c1, t3c2, t3c3, t3c4) = hal::pwm::tim3(dp.TIM3, 16000, 100.Hz(), &clocks);
    let mut throttle = t2c1.output_to_pa5(gpioa.pa5.into_af1_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl));
    let mut aileron_r = t3c2.output_to_pa4(gpioa.pa4.into_af2_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl));
    let mut rudder = t3c1.output_to_pa6(gpioa.pa6.into_af2_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl));
    let mut elevator = t3c3.output_to_pb0(gpiob.pb0.into_af2_push_pull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl));
    let mut aileron_l = t3c4.output_to_pb1(gpiob.pb1.into_af2_push_pull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl));

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

    let mut imu = Bno055::new(bus.acquire_i2c());

    imu.init(&mut delay).unwrap();
    imu.set_external_crystal(true, &mut delay).unwrap();
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

    delay.delay_ms(500);
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

    let mut r = util::DumbFilter::new(0.0);
    let mut p = util::DumbFilter::new(0.0);
    let mut y = util::DumbFilter::new(0.0);
    let mut a = 0.0f32;

    let mut pid_r = Pid::<f32>::new(1.5, 1.0, 20.0, 1.0, 0.7, 0.3, 1.0, 0.0);
    let mut pid_p = Pid::<f32>::new(0.5, 0.1, 50.0, 0.5, 0.3, 0.5, 1.0, 0.0);
    let mut ta = 0.0f32;
    let mut pp = 0.0f32;
    let mut oa = 0.0f32;
    let mut oy = 0.0f32;
    let mut sy = 0.0f32;
    let mut state1 = 0;
    let mut state2 = 0;
    let mut state3 = 0;

    delay.delay_ms(1000);

    println!("Start");

    let mut cnt = 0;
    loop {
        const SERVO_PWM_OFFSET: u16 = (1.52 / 0.000625 - 1024.0) as u16;
        const AILERON_DIFF_RATIO: f32 = 0.6;

        let cycle = pac::DWT::get_cycle_count();

        if let Ok(quat) = imu.quaternion() {
            let (nr, np, ny) = quat_to_euler(quat);
            r.update(nr);
            p.update(np);
            y.update(ny);
            // print!("raw euler: {:4} ", nr.to_degrees() as i32);
            // print!("{:4} ", np.to_degrees() as i32);
            // print!("{:4}, ", ny.to_degrees() as i32);
            print!("roll: {:4}, ", r.value().to_degrees() as i32);
            print!("pitch: {:4}, ", p.value().to_degrees() as i32);
            print!("yaw: {:4}, ", y.value().to_degrees() as i32);
            // FIXME
            // let acc = Quat::from_rotation_y(r) * Quat::from_rotation_x(p) * Vec3::from(acc);
            // let aa = -(acc.to_array()[2] - 9.80665);
            if cnt % 10 == 0 {
                if let (Ok(pressure_o), Ok(pressure_s)) = (
                    barometer_o.pressure(),
                    barometer_s.read_pressure(),
                ) {
                    let a_o = (pressure_o - pressure_mean_o) * -0.08344407986;
                    let a_s = (pressure_s * 100.0 - pressure_mean_s) * -0.08344407986;
                    a = (a_o as f32 + a_s) / 2.0;
                }
            }
            // print!("altitude: {:4}, ", a as i32);
        }

        if let Some(t10j) = &mut t10j {
            t10j.update();
            let state = t10j.state();
            // println!("S.BUS: {:?}", state.raw());

            if let (true, start) = state.button(5) {
                // FIXME: duration, double click
                if start {
                    ejector.set_high().unwrap();
                    ejector_start_cnt = cnt;
                }
            }

            let (a, e, t, r) = if let (true, start) = state.button(6) {
                if start {
                    ta = a;
                    sy = y.value();
                    pid_r.next_control_output(r.value());
                    pid_r.reset_integral_term();
                    pid_p.next_control_output(p.value());
                    pid_p.reset_integral_term();
                }
                match state.switch(7, 3) {
                    (0, changed) => {
                        if start | changed {
                            pid_r.setpoint = core::f32::consts::FRAC_PI_4;
                        }
                        let pr = pid_r.next_control_output(r.value()).output;
                        if a - ta > 0.05 {
                            pid_p.setpoint = 0.0;
                        } else if a - ta < -0.05 {
                            pid_p.setpoint = core::f32::consts::FRAC_PI_6;
                        }
                        if cnt % 10 == 0 {
                            pp = pid_p.next_control_output(p.value()).output;
                            if (a - ta < -0.05) & (a < oa) {
                                pp = (pp + (oa - a) * 10.0).min(1.0);
                            }
                            oa = a;
                        }
                        let a = 1024.0 * pr * -0.5;
                        let e = t10j.trim(2) as f32 + 1024.0 * pp;
                        let t = 368.0 + 1312.0 * 0.275 + 656.0 * 0.1 * pp; // 0.225~0.325
                        let r = t10j.trim(4) as f32 + 1024.0 * pr * 0.6;
                        // println!(", pr: {:3}, pp: {:3}", (pr * 100.0) as i32, (pp * 100.0) as i32);
                        (a, e as u16, t as u16, r as u16)
                    },
                    (1, changed) => {
                        if start | changed {
                            state1 = 0;
                            state2 = 0;
                            state3 = 0;
                            pid_r.setpoint = core::f32::consts::FRAC_PI_4;
                        }
                        match state1 {
                            0 => {
                                // FIXME: broken when sy is around +-pi
                                if (state3 == 0) & (oy > sy) & (y.value() < sy) {
                                    state3 = 1;
                                }
                                if (state3 == 1) & (oy < core::f32::consts::FRAC_PI_6) & (y.value() > core::f32::consts::FRAC_PI_6) {
                                    state1 = 1;
                                    state2 = cnt;
                                    state3 = 0;
                                    pid_r.setpoint = 0.0;
                                }
                            },
                            1 => {
                                if cnt > state2 + 100 {
                                    state1 = 2;
                                    pid_r.setpoint = -core::f32::consts::FRAC_PI_4;
                                }
                            },
                            _ => {},
                        }
                        let pr = pid_r.next_control_output(r.value()).output;
                        if a - ta > 0.05 {
                            pid_p.setpoint = 0.0;
                        } else if a - ta < -0.05 {
                            pid_p.setpoint = core::f32::consts::FRAC_PI_6;
                        }
                        if cnt % 10 == 0 {
                            pp = pid_p.next_control_output(p.value()).output;
                            if (a - ta < -0.05) & (a < oa) {
                                pp = (pp + (oa - a) * 10.0).min(1.0);
                            }
                            oa = a;
                        }
                        let a = 1024.0 * pr * -0.5;
                        let e = t10j.trim(2) as f32 + 1024.0 * pp;
                        let t = 368.0 + 1312.0 * 0.275 + 656.0 * 0.1 * pp; // 0.225~0.325
                        let r = t10j.trim(4) as f32 + if state1 == 1 { 0.0 } else { 1024.0 * pr * 0.6 };
                        oy = y.value();
                        (a, e as u16, t as u16, r as u16)
                    },
                    (_, changed) => {
                        if start | changed {
                            state1 = 0;
                            state2 = 0;
                            state3 = 0;
                            pid_r.setpoint = -core::f32::consts::FRAC_PI_4;
                        }
                        // FIXME: broken when sy is around +-pi
                        if (state3 < 2) & (oy > sy) & (y.value() < sy) {
                            state3 += 1;
                        }
                        if state3 == 2 {
                            ta += 2.0;
                            state3 = 3;
                        }
                        let pr = pid_r.next_control_output(r.value()).output;
                        if a - ta > 0.05 {
                            pid_p.setpoint = 0.0;
                        } else if a - ta < -0.05 {
                            pid_p.setpoint = core::f32::consts::FRAC_PI_6;
                        }
                        if cnt % 10 == 0 {
                            pp = pid_p.next_control_output(p.value()).output;
                            if (a - ta < -0.05) & (a < oa) {
                                pp = (pp + (oa - a) * 10.0).min(1.0);
                            }
                            oa = a;
                        }
                        let a = 1024.0 * pr * -0.5;
                        let e = t10j.trim(2) as f32 + 1024.0 * pp;
                        let t = 368.0 + 1312.0 * 0.275 + 656.0 * 0.1 * pp; // 0.225~0.325
                        let r = t10j.trim(4) as f32 + 1024.0 * pr * 0.6;
                        oy = y.value();
                        (a, e as u16, t as u16, r as u16)
                    },
                }
            } else {
                // print!("\n");
                let a = (state.value(1) as i16 - 1024) as f32;
                (a, state.value(2), state.value(3), state.value(4))
            };
            // print!("pwm: {:?}", (a as i16, e, t, r));
            // FIXME aileron trim
            aileron_l.set_duty(2048 - (1024 + (a * if a > 0.0 { 1.0 } else { AILERON_DIFF_RATIO }) as i16) as u16 + SERVO_PWM_OFFSET);
            aileron_r.set_duty((1024 + (a * if a < 0.0 { 1.0 } else { AILERON_DIFF_RATIO }) as i16) as u16 + SERVO_PWM_OFFSET);
            elevator.set_duty(2048 - e + SERVO_PWM_OFFSET);
            throttle.set_duty((t + SERVO_PWM_OFFSET) as u32);
            rudder.set_duty(r + SERVO_PWM_OFFSET);
        }

        if cnt - ejector_start_cnt > 50 {
            ejector.set_low().unwrap();
        }

        cnt += 1;
        println!("took {} cycles", pac::DWT::get_cycle_count() - cycle);

        sys_clock.wait();
    }
}

fn quat_to_euler(quat: mint::Quaternion<f32>) -> (f32, f32, f32) {
    let quat = Quat::from(quat);
    // roll: Y, pitch: X, yaw: Z for BNO055
    // roll: Z, pitch: X, yaw: Y for glam
    // let (roll, yaw, pitch) = quat.to_euler(EulerRot::YXZ);
    // let (pitch, yaw, roll) = quat.to_euler(EulerRot::ZXY);
    // let (yaw, pitch, roll) = quat.to_euler(EulerRot::XZY);
    let (roll, pitch, yaw) = quat.to_euler(EulerRot::YZX);
    let roll = -roll;
    let pitch = pitch;
    let yaw = if yaw < 0.0 { yaw + core::f32::consts::PI } else { yaw - core::f32::consts::PI };
    (roll, pitch, yaw)
}

#[cfg(debug_assertions)]
#[panic_handler]
pub fn panic(info: &core::panic::PanicInfo) -> ! {
    println!("{}", info);
    loop {}
}
