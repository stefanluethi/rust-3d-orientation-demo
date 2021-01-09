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
use shared_bus_rtic::SharedBus;

use rtic::app;
use rtic::cyccnt::U32Ext;

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
use stm32f4xx_hal::stm32::I2C1;
use embedded_hal::digital::OutputPin;

const DELAY_MS: u32 = 100;
const DELAY_TICKS: u32 = 4_800_000;

pub struct SharedBusResources<T> where T: 'static {
    sensor_imu: Lsm6dso<SharedBus<T>>,
    sensor_temp: Stts751<SharedBus<T>>,
}

use stm32f4xx_hal::{
    i2c::I2c,
    gpio,
    gpio::{
        AlternateOD,
        gpioa,
        gpiob,
    },
    serial::Tx
};
type BusType = hal::i2c::I2c<stm32::I2C1, (gpiob::PB8<AlternateOD<gpio::AF4>>, gpiob::PB9<AlternateOD<gpio::AF4>>)>;

// for some reason the peripheral access crate in the F4 hal is called `stm32`
#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        tx: Tx<stm32f4xx_hal::stm32::USART2>,
        bus_devices: SharedBusResources<BusType>,
        angle_gyro: F32x3,
        led: gpioa::PA5<gpio::Output<gpio::PushPull>>,
    }

    #[init(schedule = [sense_orientation, heartbeat])]
    fn init(cx: init::Context) -> init::LateResources {
        let mut core = cx.core;
        core.DWT.enable_cycle_counter();

        // Setup clocks
        let mut rcc = cx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.mhz()).freeze();

        // gpio's
        let gpioa = cx.device.GPIOA.split();
        let gpiob = cx.device.GPIOB.split();
        let gpioc = cx.device.GPIOC.split();

        // uart
        let txd = gpioa.pa2.into_alternate_af7();
        let rxd = gpioa.pa3.into_alternate_af7();
        let serial = hal::serial::Serial::usart2(
            cx.device.USART2,
            (txd, rxd),
            hal::serial::config::Config::default().baudrate(115_200.bps()),
            clocks
        ).unwrap();
        let (mut tx, _rx) = serial.split();


        // button to led map module
        let led = gpioa.pa5.into_push_pull_output();
        let button = gpioc.pc13.into_floating_input();
        //let mut button_led_mapper = ButtonLedHandlerImplementation::new(led, button);

        // sensors
        let sda = gpiob.pb9.into_alternate_af4_open_drain();
        let scl = gpiob.pb8.into_alternate_af4_open_drain();
        let i2c = hal::i2c::I2c::i2c1(
            cx.device.I2C1,
            (scl, sda),
            KiloHertz(100),
            clocks
        );
        // create a shared bus, because we have two sensor drivers accessing the bus
        let sensor_bus = shared_bus_rtic::new!(i2c, BusType);

        let mut sensor_imu = Lsm6dso::new(
            sensor_bus.acquire(),
            SlaveAddr::Alternate)
            .expect("LSM3DSO could not be initialized");
        sensor_imu.set_acc_range(lsm6dso::AccRange::G2).unwrap();
        sensor_imu.set_gyro_range(lsm6dso::GyroRange::DPS250).unwrap();

        let sensor_temp = Stts751::new(sensor_bus.acquire())
            .expect("STTS751 could not be initialized");

        let mut angle_gyro = F32x3::new(0., 0., 0.);

        // cx.schedule.sense_temp(cx.start + DELAY_TICKS.cycles()).unwrap();
        cx.schedule.sense_orientation(cx.start + DELAY_TICKS.cycles()).unwrap();
        cx.schedule.heartbeat(cx.start + DELAY_TICKS.cycles()).unwrap();

        init::LateResources {
            tx: tx,
            bus_devices: SharedBusResources { sensor_imu, sensor_temp },
            angle_gyro: angle_gyro,
            led: led,
        }
    }

    // #[task(resources = [bus_devices, tx, angle_gyro], schedule = [sense_temp])]
    // fn sense_temp(cx: sense_temp::Context) {
    //     let temp = cx.resources.bus_devices.sensor_temp.temp_norm().unwrap();
    //     writeln!(cx.resources.tx,
    //              "{{\
    //              \"meas\":\"temp\",\
    //              \"values\":{{\
    //                  \"T\":{}\
    //              }},\
    //              \"unit\":\"°C\"\
    //          }}",
    //              temp
    //     ).unwrap();
    //
    //     cx.schedule.sense_temp(cx.scheduled + DELAY_TICKS.cycles()).unwrap();
    // }

    #[task(resources = [bus_devices, tx, angle_gyro], schedule = [sense_orientation])]
    fn sense_orientation(mut cx: sense_orientation::Context) {
        let acceleration = cx.resources.bus_devices.sensor_imu.accel_norm().unwrap();
        let angular_velocity = cx.resources.bus_devices.sensor_imu.gyro_norm().unwrap();
        let mut angle = calc_angles(&acceleration, &angular_velocity, &mut cx.resources.angle_gyro);

        // print measurements and orientation in JSON format
        writeln!(cx.resources.tx,
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

        writeln!(cx.resources.tx,
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

        writeln!(cx.resources.tx,
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

        cx.schedule.sense_orientation(cx.scheduled + DELAY_TICKS.cycles()).unwrap();
    }

    #[task(resources = [led], schedule = [heartbeat])]
    fn heartbeat(cx: heartbeat::Context) {
        cx.resources.led.toggle();
        cx.schedule.heartbeat(cx.scheduled + DELAY_TICKS.cycles()).unwrap();
    }

    // external interrupt we use for our tasks
    extern "C" {
        fn EXTI0();
    }
};

/// calculate 3D orientation on degrees from angular velocity (°/s) and
/// acceleration (g)
fn calc_angles(acc: &F32x3, gyro: &F32x3, angle_gyro: &mut F32x3) -> vector::F32x3 {
    const TIME_STEP: f32 = DELAY_MS as f32 /1000.;
    const DEG_PER_RAD: f32 = 57.2958;
    let beta = 0.05;

    let mut angle_acc = F32x3::new(0., 0., 0.);
    angle_acc.x = - F32Ext::atan2(
        acc.y,
        F32Ext::sqrt(acc.x.powi(2) + acc.z.powi(2))
    ) * DEG_PER_RAD;
    angle_acc.y = 90. - F32Ext::atan2(
        acc.z,
        F32Ext::sqrt(acc.x.powi(2) + acc.y.powi(2))
    ) * DEG_PER_RAD;
    angle_acc.z = F32Ext::atan2(
        acc.x,
        F32Ext::sqrt(acc.y.powi(2) + acc.z.powi(2))
    ) * DEG_PER_RAD;

    angle_gyro.x += -gyro.x * TIME_STEP;
    angle_gyro.y += -gyro.y * TIME_STEP;
    angle_gyro.z += gyro.z * TIME_STEP;

    // calculate weighted angle from the two sensors
    F32x3::new(
        angle_acc.x * beta + (1.-beta) * angle_gyro.x,
        angle_acc.y * beta + (1.-beta) * angle_gyro.y,
        angle_acc.z * beta + (1.-beta) * angle_gyro.z,
    )
}