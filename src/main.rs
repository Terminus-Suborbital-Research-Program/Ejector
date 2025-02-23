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

use defmt_rtt as _; // global logger

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
        actuators::servo::EjectorServo,
        communications::hc12::{UART1Bus, GPIO10},
        phases::EjectorStateMachine,
    };

    use super::*;

    use communications::{link_layer::LinkLayerDevice, *};

    use hal::gpio::{self, FunctionSio, PullNone, SioOutput};
    use rp235x_hal::uart::UartPeripheral;
    pub const XTAL_FREQ_HZ: u32 = 12_000_000u32;

    use usb_device::{class_prelude::*, prelude::*};
    use usbd_serial::SerialPort;

    use hc12::HC12;

    use rtic_sync::channel::{Receiver, Sender};
    use serial_handler::{HEAPLESS_STRING_ALLOC_LENGTH, MAX_USB_LINES};

    pub type UART0Bus = UartPeripheral<
        rp235x_hal::uart::Enabled,
        rp235x_hal::pac::UART0,
        (
            gpio::Pin<gpio::bank0::Gpio0, gpio::FunctionUart, gpio::PullDown>,
            gpio::Pin<gpio::bank0::Gpio1, gpio::FunctionUart, gpio::PullDown>,
        ),
    >;

    pub static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

    #[shared]
    pub struct Shared {
        pub ejector_servo: EjectorServo,
        pub radio_link: LinkLayerDevice<HC12<UART1Bus, GPIO10>>,
        pub usb_serial: SerialPort<'static, hal::usb::UsbBus>,
        pub usb_device: UsbDevice<'static, hal::usb::UsbBus>,
        pub serial_console_writer: serial_handler::SerialWriter,
        pub clock_freq_hz: u32,
        pub state_machine: EjectorStateMachine,
        pub blink_status_delay_millis: u64,
        pub suspend_packet_handler: bool,
    }

    #[local]
    pub struct Local {
        pub led: gpio::Pin<gpio::bank0::Gpio25, FunctionSio<SioOutput>, PullNone>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local) {
        startup::startup(ctx)
    }

    extern "Rust" {
        // Takes care of receiving incoming packets
        #[task(shared = [radio_link, state_machine, suspend_packet_handler], priority = 1)]
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
        #[task(shared=[serial_console_writer, radio_link, clock_freq_hz, ejector_servo, state_machine, suspend_packet_handler], priority = 2)]
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

        // An async task to program the HC12 module
        #[task(shared = [radio_link, serial_console_writer, suspend_packet_handler], priority = 3)]
        async fn hc12_programmer(mut ctx: hc12_programmer::Context);
    }
}
