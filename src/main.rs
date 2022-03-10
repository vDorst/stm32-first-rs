#![no_main]
#![no_std]

use defmt_rtt as _;
// global logger
use panic_probe as _;
use stm32f4xx_hal as hal;

use hal::otg_fs::{UsbBus, USB};
use usb_device::prelude::*;

use cortex_m_rt::entry;
use hal::{gpio::NoPin, pac, prelude::*, spi::{Polarity, Phase, Mode}};

use smart_leds::{brightness, hsv::RGB8, SmartLedsWrite};
use ws2812_spi as ws2812;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

static mut EP_MEMORY: [u32; 1024] = [0; 1024];
const NUM_LEDS: usize = 64;

#[entry]
fn main() -> ! {
    if let (Some(dp), Some(_cp)) = (
        pac::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        // Set up the LED. On the Nucleo-446RE it's connected to pin PA5.
        let gpioc = dp.GPIOC.split();
        let mut led = gpioc.pc13.into_push_pull_output();

        // Set up the system clock. We want to run at 48MHz for this one.
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(56.MHz()).require_pll48clk().freeze();

        // Create a delay abstraction based on SysTick
        let mut delay = dp.TIM1.delay_us(&clocks);

        let gpioa = dp.GPIOA.split();

        let spi_mode: Mode = Mode {
            polarity: Polarity::IdleLow,
            // the default WS2812::MODE seems to add a delay between every 8 bits
            // which cause the ws2812 led produce the wrong colours.
            phase: Phase::CaptureOnSecondTransition,
        };

        // SPI PIN, (CLK, MISO, MOSI)
        let spi = dp.SPI1.spi(
            (gpioa.pa5, NoPin, gpioa.pa7),
            spi_mode,
            3500.kHz(),
            &clocks,
        );        

        // let LED = gpioa.pa5;

        let usb = USB {
            usb_global: dp.OTG_FS_GLOBAL,
            usb_device: dp.OTG_FS_DEVICE,
            usb_pwrclk: dp.OTG_FS_PWRCLK,
            pin_dm: gpioa.pa11.into_alternate(),
            pin_dp: gpioa.pa12.into_alternate(),
            hclk: clocks.hclk(),
        };

        let usb_bus = UsbBus::new(usb, unsafe { &mut EP_MEMORY });

        let mut serial = usbd_serial::SerialPort::new(&usb_bus);
    
        let mut usb_dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Real Fake company")
            .product("Serial port")
            .serial_number("#12343")
            .device_class(usbd_serial::USB_CLASS_CDC)
            .build();          

        let mut ws = ws2812::Ws2812::new(spi);

        let led_off: RGB8 = (0x00, 0x00, 0x00).into();
        let mut data = [led_off; NUM_LEDS];

               
        let bla = ws.write(data.iter().cloned());
        if let Err(e) = bla {
            match e {
                hal::spi::Error::Overrun => defmt::println!("Overrun"),
                hal::spi::Error::ModeFault => defmt::println!("ModeFault"),
                hal::spi::Error::Crc => defmt::println!("Crc"),
                _ => defmt::println!("_ Unknown"),
            }
        }
        
        // Wait before start write for syncronization
        delay.delay(200.micros());
        
        led.set_low();

        // loop {
        //     delay.delay(1000.millis());
        //     led.set_low();
        //     delay.delay(1000.millis());
        //     led.set_high();

        // }

        let mut j: u16 = 0;

        loop {
            j += 1;
            if j == (256 * 5) { j = 0; }
            
            for (i, b) in data.iter_mut().enumerate() {
                *b = wheel((((i * 256) as u16 / NUM_LEDS as u16 + j as u16) & 255) as u8);
            }
            ws.write(brightness(data.iter().cloned(), 0x10)).ok();

                
            if !usb_dev.poll(&mut [&mut serial]) {
                delay.delay(10.millis());
                continue;
            }
    
            let mut buf = [0u8; 64];
    
            match serial.read(&mut buf) {
                Ok(count) if count > 0 => {
                    // Echo back in upper case
                    for c in buf[0..count].iter_mut() {
                        if 0x61 <= *c && *c <= 0x7a {
                            *c &= !0x20;
                        }
                    }
    
                    let mut write_offset = 0;
                    while write_offset < count {
                        match serial.write(&buf[write_offset..count]) {
                            Ok(len) if len > 0 => {
                                write_offset += len;
                            }
                            _ => {}
                        }
                    }
                }
                _ => {}
            }
        }
    }

    loop {}
}

/// Input a value 0 to 255 to get a color value
/// The colours are a transition r - g - b - back to r.
fn wheel(mut wheel_pos: u8) -> RGB8 {
    wheel_pos = 255 - wheel_pos;
    if wheel_pos < 85 {
        return (255 - wheel_pos * 3, 0, wheel_pos * 3).into();
    }
    if wheel_pos < 170 {
        wheel_pos -= 85;
        return (0, wheel_pos * 3, 255 - wheel_pos * 3).into();
    }
    wheel_pos -= 170;
    (wheel_pos * 3, 255 - wheel_pos * 3, 0).into()
}