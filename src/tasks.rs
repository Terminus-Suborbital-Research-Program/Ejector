use core::cmp::max;

use bin_packets::{packets::ApplicationPacket, phases::EjectorPhase};
use embedded_hal::digital::StatefulOutputPin;
use fugit::ExtU64;
use rtic::Mutex;
use rtic_monotonics::Monotonic;

use crate::{
    app::*,
    communications::{
        hc12::BaudRate,
        link_layer::{LinkLayerPayload, LinkPacket},
    },
    Mono,
};

pub async fn incoming_packet_handler(mut ctx: incoming_packet_handler::Context<'_>) {
    loop {
        while let Some(packet) = ctx.shared.radio_link.lock(|radio| radio.read_link_packet()) {
            // Only act on packets with valid checksums
            if !packet.verify_checksum() {
                continue;
            }

            match packet.payload {
                LinkLayerPayload::Payload(app_packet) => {
                    // Ejector only handles commands right now
                    match app_packet {
                        ApplicationPacket::Command(command) => {
                            // Enter phase, based on the command
                            match command {
                                bin_packets::packets::CommandPacket::EjectorPhaseSet(phase) => {
                                    ctx.shared.state_machine.lock(|state_machine| {
                                        state_machine.set_phase(phase);
                                    });

                                    // Send a response, with no data (for my testing)
                                    ctx.shared.radio_link.lock(|radio| {
                                        let packet = LinkPacket::default();
                                        radio.write_link_packet(packet).ok();
                                    });
                                }

                                // Ping commands return an empty packet
                                bin_packets::packets::CommandPacket::Ping => {
                                    ctx.shared.radio_link.lock(|radio| {
                                        let packet = LinkPacket::default();
                                        radio.write_link_packet(packet).ok();
                                    });
                                }

                                _ => {
                                    // Unhandled command on the Ejector
                                }
                            }
                        }

                        _ => {}
                    }
                }

                _ => {
                    // Link not implimented
                }
            }
        }

        Mono::delay(10_u64.millis()).await;
    }
}

pub async fn heartbeat(mut ctx: heartbeat::Context<'_>) {
    loop {
        _ = ctx.local.led.toggle();

        Mono::delay(
            ctx.shared
                .blink_status_delay_millis
                .lock(|delay| *delay)
                .millis(),
        )
        .await;
    }
}

pub fn uart_interrupt(mut ctx: uart_interrupt::Context<'_>) {
    ctx.shared.radio_link.lock(|radio| {
        radio.device.update().ok();
    });
}

pub async fn state_machine_update(mut ctx: state_machine_update::Context<'_>) {
    loop {
        let wait_time = ctx.shared.state_machine.lock(|state_machine| {
            let wait_time = state_machine.transition();
            wait_time
        });

        match ctx
            .shared
            .state_machine
            .lock(|state_machine| state_machine.phase())
        {
            EjectorPhase::Standby => {
                // Hold the deployable
                ctx.shared.ejector_servo.lock(|servo| {
                    servo.hold();
                });

                // 1000ms delay
                ctx.shared
                    .blink_status_delay_millis
                    .lock(|delay| *delay = 1000);
            }

            EjectorPhase::Ejection => {
                // Eject the deployable
                ctx.shared.ejector_servo.lock(|servo| {
                    servo.eject();
                });
                // 200ms delay
                ctx.shared
                    .blink_status_delay_millis
                    .lock(|delay| *delay = 200);
            }

            EjectorPhase::Hold => {
                // Hold the deployable
                ctx.shared.ejector_servo.lock(|servo| {
                    servo.hold();
                });
                // 5000ms delay
                ctx.shared
                    .blink_status_delay_millis
                    .lock(|delay| *delay = 5000);
            }
        }

        // We should never wait less than 1ms, tbh
        Mono::delay(max(wait_time, 1).millis()).await;
    }
}

pub async fn radio_flush(mut ctx: radio_flush::Context<'_>) {
    let mut on_board_baudrate: BaudRate = BaudRate::B9600;
    let bytes_to_flush = 16;

    loop {
        ctx.shared.radio_link.lock(|radio| {
            radio.device.flush(bytes_to_flush).ok();
            on_board_baudrate = radio.device.get_baudrate();
        });

        // Need to wait wait the in-air baudrate, or the on-board baudrate
        // whichever is slower

        let mut slower =
            core::cmp::min(on_board_baudrate.to_u32(), on_board_baudrate.to_in_air_bd());

        // slower is bps, so /1000 to get ms
        slower = slower / 1000;

        // Delay for that times the number of bytes flushed
        Mono::delay((slower as u64 * bytes_to_flush as u64).millis()).await;
    }
}
