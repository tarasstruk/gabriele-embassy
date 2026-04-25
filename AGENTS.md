# AGENTS.md

## Project Overview

Bare-metal Rust firmware for RP2040 (Raspberry Pi Pico W) that controls a **Gabriele typewriter** over WiFi. Accepts TCP connections on port 1234, translates text into typewriter instructions, and sends them via UART to the machine. Uses the Embassy async runtime (`#![no_std]`, `#![no_main]`).

## Architecture

- **`src/main.rs`** — Entry point. Sets up UART (4800 baud on UART1), WiFi (CYW43 via PIO0), PIO1 for typewriter pulse detection, and a TCP server loop. Coordinates everything through static `Signal` channels (`INPUT`, `ECHO`, `SIGNAL`, `UART_READY`).
- **`src/tasks.rs`** — Embassy tasks: `uart_tx_task` (byte send → wait for confirmation pulse → echo back), `pio_task_sm0` (watches typewriter confirmation), `cyw43_task`, `net_task`.
- **`src/feedback.rs`** — UART RX protocol: reads 2-byte acknowledgments (`0xA1 0xA2` = Started, `0xA3 0xA0` = Stopped).
- **`src/setup_pio_1.rs`** — PIO1 program that detects typewriter confirmation pulses on pins 3 and 4.
- **`gabriele`** — External local crate (`../gabriele/gabriele`) providing `Machine`, `Symbol`, `Instruction`, and daisywheel definitions. Must exist as a sibling directory.

## Data Flow

```
TCP client → socket.read → INPUT signal → uart_tx_task → UART TX → typewriter
typewriter pulse → PIO1 → SIGNAL → uart_tx_task → ECHO signal → socket.write (echo back)
```

The TCP interface is a **raw byte passthrough** — the client sends the typewriter's native UART protocol bytes, not text. No character-to-instruction translation is performed on the TCP stream. The only text-level printing happens at connection startup, where `Machine::print()` outputs a greeting and IP address.

Start/stop sequences (`0xA1 0x00 0xA2 0x00` / `0xA3 0x00 0xA0 0x00`) bracket each TCP session, with UART RX feedback confirming machine readiness.

## Build & Flash

```sh
# Requires: cargo, probe-rs, thumbv6m-none-eabi target
# The sibling `../gabriele/gabriele` crate must be present
cargo build          # cross-compiles for thumbv6m-none-eabi (RP2040)
cargo run            # builds + flashes via probe-rs (see .cargo/config.toml if present)
```

- **WiFi credentials**: stored in plaintext files `wifi_net` and `wifi_pass` at project root (included at compile time via `include_bytes!`).
- **CYW43 firmware**: vendored in `cyw43-firmware/`, included at compile time.
- **Linker config**: `memory.x` defines flash/RAM layout; `build.rs` copies it and sets linker args (`-Tlink.x -Tlink-rp.x -Tdefmt.x`).

## Key Conventions

- **No heap**: `#![no_std]` with `heapless` collections. All statics use `StaticCell` for one-time init.
- **Async coordination**: `embassy_sync::Signal<ThreadModeRawMutex, T>` for inter-task communication (not channels/queues).
- **Logging**: `defmt` + `defmt-rtt` (not `println!`). Use `info!`, `warn!`, `error!` macros.
- **Pin assignments**: UART1 on pins 4/5, RTS on pin 7, PIO1 reads pins 3/4, WiFi on pins 23/24/25/29.
- **Edition 2024** Rust with `embassy-executor` task spawning pattern.

## Gotchas

- The `gabriele` dependency is a **local path** (`../gabriele/gabriele`) — the workspace won't build without it.
- WiFi credentials files (`wifi_net`, `wifi_pass`) must exist or compilation fails.
- This is single-core `thumbv6m-none-eabi` — no `std`, no threads, no allocator.

