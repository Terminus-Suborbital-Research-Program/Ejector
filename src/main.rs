#![no_std]
#![no_main]

// Our Modules
pub mod actuators;
pub mod communications;
pub mod phases;
pub mod utilities;

// RTIC Tasks
pub mod startup;
pub mod tasks;
pub mod usb_tasks;

use tasks::*;
use usb_tasks::*;

use panic_halt as _;

// HAL Access
use rp235x_hal as hal;

// Monotonics
use rtic_monotonics::rp235x::prelude::*;
rp235x_timer_monotonic!(Mono);

/// Tell the Boot ROM about our application
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: rp235x_hal::block::ImageDef = rp235x_hal::block::ImageDef::secure_exe();

#[rtic::app(
    device = hal::pac,
    dispatchers = [PIO2_IRQ_0, PIO2_IRQ_1, DMA_IRQ_0],
)]
mod app {
    use crate::{
        actuators::servo::{EjectionServoMosfet, EjectorServo, Servo},
        communications::{
            hc12::{UART1Bus, GPIO10},
            link_layer::Device,
        },
        phases::EjectorStateMachine,
    };

    use super::*;

    

    use communications::{
        link_layer::LinkLayerDevice,
        serial_handler::HeaplessString,
        *,
    };

    
    use embedded_hal::digital::OutputPin;
    use fugit::RateExtU32;
    use hal::{
        gpio::{self, FunctionSio, PullNone, SioOutput},
        sio::Sio,
    };
    use rp235x_hal::{
        clocks::init_clocks_and_plls,
        pwm::Slices,
        uart::{DataBits, StopBits, UartConfig, UartPeripheral},
        Clock, Watchdog,
    };
    const XTAL_FREQ_HZ: u32 = 12_000_000u32;

    use usb_device::{class_prelude::*, prelude::*};
    use usbd_serial::{embedded_io::Write, SerialPort};

    use hc12::HC12;

    use rtic_sync::{
        channel::{Receiver, Sender},
        make_channel,
    };
    use serial_handler::{HEAPLESS_STRING_ALLOC_LENGTH, MAX_USB_LINES};

    pub type UART0Bus = UartPeripheral<
        rp235x_hal::uart::Enabled,
        rp235x_hal::pac::UART0,
        (
            gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionUart, gpio::PullDown>,
            gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionUart, gpio::PullDown>,
        ),
    >;

    static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

    

    #[shared]
    struct Shared {
        pub ejector_servo: EjectorServo,
        pub radio_link: LinkLayerDevice<HC12<UART1Bus, GPIO10>>,
        pub usb_serial: SerialPort<'static, hal::usb::UsbBus>,
        pub usb_device: UsbDevice<'static, hal::usb::UsbBus>,
        pub serial_console_writer: serial_handler::SerialWriter,
        pub clock_freq_hz: u32,
        pub state_machine: EjectorStateMachine,
        pub blink_status_delay_millis: u64,
    }

    #[local]
    struct Local {
        pub led: gpio::Pin<gpio::bank0::Gpio25, FunctionSio<SioOutput>, PullNone>,
    }

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        // Reset the spinlocks - this is skipped by soft-reset
        unsafe {
            hal::sio::spinlock_reset();
        }

        // Channel for sending strings to the USB console
        let (usb_console_line_sender, usb_console_line_receiver) =
            make_channel!(HeaplessString, MAX_USB_LINES);

        // Channel for incoming commands from the USB console
        let (usb_console_command_sender, usb_console_command_receiver) =
            make_channel!(HeaplessString, MAX_USB_LINES);

        // Set up clocks
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);
        let clocks = init_clocks_and_plls(
            XTAL_FREQ_HZ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut ctx.device.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        Mono::start(ctx.device.TIMER0, &ctx.device.RESETS);

        // The single-cycle I/O block controls our GPIO pins
        let sio = Sio::new(ctx.device.SIO);

        // Set the pins to their default state
        let bank0_pins = hal::gpio::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );

        // Configure GPIO25 as an output
        let mut led_pin = bank0_pins
            .gpio25
            .into_pull_type::<PullNone>()
            .into_push_pull_output();
        led_pin.set_low().unwrap();
        // Start the heartbeat task
        heartbeat::spawn().ok();

        // Get clock frequency
        let clock_freq = clocks.peripheral_clock.freq();

        // Pin setup for UART1
        let uart1_pins = (
            bank0_pins.gpio8.into_function(),
            bank0_pins.gpio9.into_function(),
        );
        let mut uart1_peripheral =
            UartPeripheral::new(ctx.device.UART1, uart1_pins, &mut ctx.device.RESETS)
                .enable(
                    UartConfig::new(9600.Hz(), DataBits::Eight, None, StopBits::One),
                    clocks.peripheral_clock.freq(),
                )
                .unwrap();
        uart1_peripheral.enable_rx_interrupt(); // Make sure we can drive our interrupts

        // Use pin 14 (GPIO10) as the HC12 configuration pin
        let hc12_configure_pin = bank0_pins.gpio10.into_push_pull_output();
        let hc12 = HC12::new(uart1_peripheral, hc12_configure_pin).unwrap();
        let radio_link = LinkLayerDevice::new(hc12, Device::Ejector);

        // Servo
        let pwm_slices = Slices::new(ctx.device.PWM, &mut ctx.device.RESETS);
        let mut ejection_pwm = pwm_slices.pwm0;
        ejection_pwm.enable();
        ejection_pwm.set_div_int(48);
        // Pin for servo mosfet digital
        let mut mosfet_pin: EjectionServoMosfet = bank0_pins.gpio1.into_push_pull_output();
        mosfet_pin.set_low().unwrap();
        let mut channel_a = ejection_pwm.channel_a;
        let channel_pin = channel_a.output_to(bank0_pins.gpio0);
        channel_a.set_enabled(true);
        let ejection_servo = Servo::new(channel_a, channel_pin, mosfet_pin);
        // Create ejector servo
        let mut ejector_servo: EjectorServo = EjectorServo::new(ejection_servo);
        ejector_servo.enable();
        ejector_servo.hold();

        // Set up USB Device allocator
        let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
            ctx.device.USB,
            ctx.device.USB_DPRAM,
            clocks.usb_clock,
            true,
            &mut ctx.device.RESETS,
        ));
        unsafe {
            USB_BUS = Some(usb_bus);
        }
        #[allow(static_mut_refs)]
        let usb_bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

        let serial = SerialPort::new(usb_bus_ref);
        let usb_dev = UsbDeviceBuilder::new(usb_bus_ref, UsbVidPid(0x16c0, 0x27dd))
            .strings(&[StringDescriptors::default()
                .manufacturer("UAH TERMINUS PROGRAM")
                .product("Canonical Toolchain USB Serial Port")
                .serial_number("TEST")])
            .unwrap()
            .device_class(2)
            .build();

        // Serial Writer Structure
        let serial_console_writer = serial_handler::SerialWriter::new(usb_console_line_sender);

        usb_serial_console_printer::spawn(usb_console_line_receiver).ok();
        usb_console_reader::spawn(usb_console_command_sender).ok();
        #[cfg(debug_assertions)]
        command_handler::spawn(usb_console_command_receiver).ok();
        radio_flush::spawn().ok();
        //incoming_packet_handler::spawn().ok();
        state_machine_update::spawn().ok();

        (
            Shared {
                //uart0: uart0_peripheral,
                //uart0_buffer,
                radio_link,
                ejector_servo,
                usb_device: usb_dev,
                usb_serial: serial,
                serial_console_writer,
                clock_freq_hz: clock_freq.to_Hz(),
                state_machine: EjectorStateMachine::new(),
                blink_status_delay_millis: 1000,
            },
            Local { led: led_pin },
        )
    }

    extern "Rust" {
        // Takes care of receiving incoming packets
        #[task(shared = [radio_link, state_machine], priority = 3)]
        async fn incoming_packet_handler(mut ctx: incoming_packet_handler::Context);

        // State machine update
        #[task(shared = [state_machine, serial_console_writer, ejector_servo, blink_status_delay_millis], priority = 1)]
        async fn state_machine_update(mut ctx: state_machine_update::Context);

        // Heartbeats the main led
        #[task(local = [led], shared = [blink_status_delay_millis], priority = 2)]
        async fn heartbeat(mut ctx: heartbeat::Context);

        // Reads from the USB console
        #[task(priority = 3, shared = [usb_device, usb_serial, serial_console_writer])]
        async fn usb_console_reader(
            mut ctx: usb_console_reader::Context,
            mut command_sender: Sender<
                'static,
                heapless::String<HEAPLESS_STRING_ALLOC_LENGTH>,
                MAX_USB_LINES,
            >,
        );

        // Writes to the USB console
        #[task(priority = 3, shared = [usb_device, usb_serial])]
        async fn usb_serial_console_printer(
            mut ctx: usb_serial_console_printer::Context,
            mut reciever: Receiver<
                'static,
                heapless::String<HEAPLESS_STRING_ALLOC_LENGTH>,
                MAX_USB_LINES,
            >,
        );

        // Command Handler for USB Console
        #[task(shared=[serial_console_writer, radio_link, clock_freq_hz, ejector_servo, state_machine], priority = 3)]
        async fn command_handler(
            mut ctx: command_handler::Context,
            mut reciever: Receiver<
                'static,
                heapless::String<HEAPLESS_STRING_ALLOC_LENGTH>,
                MAX_USB_LINES,
            >,
        );

        // Updates the radio module on the serial interrupt
        #[task(binds = UART1_IRQ, shared = [radio_link, serial_console_writer])]
        fn uart_interrupt(mut ctx: uart_interrupt::Context);

        // Radio Flush Task
        #[task(shared = [radio_link], priority = 1)]
        async fn radio_flush(mut ctx: radio_flush::Context);
    }
}
