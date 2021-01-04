#![deny(unsafe_code)]
#![no_main]
#![no_std]

#[allow(unused_extern_crates)]
// Halt on panic and print the stack trace to SWO
extern crate panic_itm;

use cortex_m;
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;
use crate::hal::{prelude::*, stm32};
use hal::time::KiloHertz;

use micromath::{F32Ext, vector, vector::F32x3};
use core::fmt::Write;

// custom module to test abstractions
mod button_led_handler_implementation;
use button_led_handler_implementation::ButtonLedHandlerImplementation;
mod button_led_handler;
use button_led_handler::ButtonLedHandler;

// custom sensor implementations
use lsm6dso::{Lsm6dso, SlaveAddr, Accelerometer, Gyro};
//use lis2mdl::{Lis2mdl, Magnetometer}; // hardware not working
use stts751::{Stts751, Temperature};
use kalman_nostd::{Kalman};

#[entry]
fn main() -> ! {
    // Take hardware peripherals
    let stm32_peripherals = stm32::Peripherals::take().expect("cannot take stm32 peripherals");
    let mut cortex_peripherals = cortex_m::peripheral::Peripherals::take().expect("cannot take cortex peripherals");

    // delay
    // Set up the system clock. We want to run at 48MHz for this one.
    let rcc = stm32_peripherals.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

    // Create a delay abstraction based on SysTick
    let mut delay = hal::delay::Delay::new(cortex_peripherals.SYST, clocks);

    // gpio's
    let gpioa = stm32_peripherals.GPIOA.split();
    let gpiob = stm32_peripherals.GPIOB.split();
    let gpioc = stm32_peripherals.GPIOC.split();

    // uart
    let txd = gpioa.pa2.into_alternate_af7();
    let rxd = gpioa.pa3.into_alternate_af7();
    let serial = hal::serial::Serial::usart2(
        stm32_peripherals.USART2,
        (txd, rxd),
        hal::serial::config::Config::default().baudrate(115_200.bps()),
        clocks
    ).unwrap();
    let (mut tx, _rx) = serial.split();

    // itm output
    let stim = &mut cortex_peripherals.ITM.stim[1];

    // button to led map module
    let led = gpioa.pa5.into_push_pull_output();
    let button = gpioc.pc13.into_floating_input();
    let mut button_led_mapper = ButtonLedHandlerImplementation::new(led, button);

    // sensors
    let sda = gpiob.pb9.into_alternate_af4_open_drain();
    let scl = gpiob.pb8.into_alternate_af4_open_drain();
    let i2c = hal::i2c::I2c::i2c1(
        stm32_peripherals.I2C1,
        (scl, sda),
        KiloHertz(100),
        clocks
    );
    // create a shared bus, because we have two sensor drivers accessing the bus
    let sensor_bus = shared_bus::BusManagerSimple::new(i2c);

    let mut sensor_imu = Lsm6dso::new(
        sensor_bus.acquire_i2c(),
        SlaveAddr::Alternate)
        .expect("LSM3DSO could not be initialized");
    sensor_imu.set_acc_range(lsm6dso::AccRange::G2).unwrap();
    sensor_imu.set_gyro_range(lsm6dso::GyroRange::DPS250).unwrap();

    let mut sensor_temp = Stts751::new(sensor_bus.acquire_i2c())
        .expect("STTS751 could not be initialized");
    cortex_m::iprintln!(stim, "init. sensor finished");

    // Kalman filter
    let x_init: [f32; 2] = [0., 0.];
    let p_init = [[0.0000001, 0.], [0., 0.0000001]];
    let k_init = [[0.], [0.]];
    let a = [[1., -0.001], [0., 1.]];
    let c  = [[1., 0.]];
    let q_w = [[0., 0.], [0., 0.]];
    let q_v = [[0.]];
    let mut kalman = Kalman::new(x_init, a, c, q_w, q_v, p_init, k_init);

    let y = [0.];
    kalman.update_step(y);

    let mut angle_gyro = F32x3::new(0., 0., 0.);

    loop {
        button_led_mapper.map_button_to_led();

        let acceleration = sensor_imu.accel_norm().unwrap();
        let angular_velocity = sensor_imu.gyro_norm().unwrap();
        let temp = sensor_temp.temp_norm().unwrap();
        let angle = calc_angles(&acceleration, &angular_velocity, &mut angle_gyro);

        // print measurements and orientation in JSON format
        writeln!(tx,
                 "{{\
                \"meas\":\"acc\",\
                \"values\":{{\
                    \"x\":{},\
                    \"y\":{},\
                    \"z\":{}\
                }},\
                \"unit\":\"g\"\
            }}",
                 acceleration.x,
                 acceleration.y,
                 acceleration.z
        ).unwrap();

        writeln!(tx,
                 "{{\
                     \"meas\":\"gyro\",\
                     \"values\":{{\
                         \"x\":{},\
                         \"y\":{},\
                         \"z\":{}\
                     }},\
                     \"unit\":\"dps\"\
                 }}",
                 angular_velocity.x,
                 angular_velocity.y,
                 angular_velocity.z
        ).unwrap();

        writeln!(tx,
                 "{{\
                     \"meas\":\"angle\",\
                     \"values\":{{\
                         \"x\":{},\
                         \"y\":{},\
                         \"z\":{}\
                     }},\
                     \"unit\":\"°\"\
                 }}",
                 angle.x,
                 angle.y,
                 angle.z
        ).unwrap();

        writeln!(tx,
                 "{{\
                 \"meas\":\"temp\",\
                 \"values\":{{\
                     \"T\":{}\
                 }},\
                 \"unit\":\"°C\"\
             }}",
                 temp
        ).unwrap();

        delay.delay_ms(100_u16);
    }
}

/// calculate 3D orientation on degrees from angular velocity (°/s) and
/// acceleration (g)
fn calc_angles(acc: &F32x3, gyro: &F32x3, angle_gyro: &mut F32x3) -> vector::F32x3 {
    let beta = 0.05;

    let mut angle_acc = F32x3::new(0., 0., 0.);
    angle_acc.x = - F32Ext::atan2(
        acc.y,
        F32Ext::sqrt(acc.x.powi(2) + acc.z.powi(2))
    ) * 57.2958;
    angle_acc.y = 90. - F32Ext::atan2(
        acc.z,
        F32Ext::sqrt(acc.x.powi(2) + acc.y.powi(2))
    ) * 57.2958;
    angle_acc.z = F32Ext::atan2(
        acc.x,
        F32Ext::sqrt(acc.y.powi(2) + acc.z.powi(2))
    ) * 57.2958;

    angle_gyro.x += -gyro.x * 0.1;
    angle_gyro.y += -gyro.y * 0.1;
    angle_gyro.z += gyro.z * 0.1;

    // calculate weighted angle from the two sensors
    F32x3::new(
        angle_acc.x * beta + (1.-beta) * angle_gyro.x,
        angle_acc.y * beta + (1.-beta) * angle_gyro.y,
        angle_acc.z * beta + (1.-beta) * angle_gyro.z,
    )
}