# embedded-test

Embedded Rust test firmware for an STM32-based FM radio bring-up project. The application uses the **Embassy** async framework, controls a **Si4743 / Si47xx FM receiver** over I²C, displays status and RDS data on an **SSD1306 OLED**, and generates a reference clock with PWM.

> **Naming note:** The repository is named `embedded-test`, but some historical identifiers may still use `stmf103c8tx` naming. The active configuration in this project targets **STM32F303VC**. If you are using an STM32F103C8T6, update the chip feature in `Cargo.toml` and the target in `.cargo/config.toml` accordingly.

## Overview

This repository is intended as a testbed for validating the radio subsystem and basic board bring-up:

- Async Embassy executor on STM32.
- Si4743 / Si47xx FM receiver control over I²C.
- FM seek with STC polling.
- RDS decoding for program service and radio text.
- SSD1306 OLED display over I²C.
- PWM-generated REFCLK for the radio.
- EXTI input for radio GPO interrupt.
- LED blink error codes for fault diagnosis.
- `defmt` + RTT logging.

Hardware connections and pin assignments should be verified against your board schematic before flashing.

## Features

- Embassy-based async firmware.
- Si4743 / Si47xx FM receiver driver.
- I²C communication with the radio and OLED.
- FM seek and tune-complete polling.
- RDS group decoding.
- SSD1306 display output.
- PWM reference clock generation.
- External interrupt input for radio GPO.
- LED blink error reporting.
- `defmt` logging over RTT.

## Hardware

Example hardware setup:

| Component | Interface / Connection |
|-----------|------------------------|
| MCU | STM32F303VC (Cortex-M4F) |
| FM receiver | Si4743, I²C (`0x63`) |
| OLED display | SSD1306, I²C |
| REFCLK | TIM2 PWM → Si4743 RCLK |
| Radio interrupt | EXTI0 / GPO |
| Error LED | GPIO output |
| Debug probe | ST-Link or probe-rs compatible |

> Adjust pins, peripherals, and I²C addresses in `src/main.rs` to match your hardware.

## Software prerequisites

- Rust stable toolchain.
- Target `thumbv7em-none-eabihf` for STM32F303VC:
  ```bash
  rustup target add thumbv7em-none-eabihf
  ```
- `arm-none-eabi-gcc` linker available in `PATH`.
- `probe-rs` for flashing and debugging.
- Optional: `defmt-print` for decoding RTT logs outside `probe-rs`.

For STM32F103C8T6, use an appropriate Cortex-M3 target such as `thumbv7m-none-eabi` and update the chip feature in `Cargo.toml`.

## Getting started

Clone the repository with submodules:

```bash
git clone --recursive https://github.com/Humminghead/embedded-test.git
cd embedded-test
```

If you cloned without `--recursive`:

```bash
git submodule update --init --recursive
```

Build the firmware:

```bash
cargo build --release
```

Flash and run with `probe-rs`:

```bash
probe-rs run --chip STM32F303VC target/thumbv7em-none-eabihf/release/<binary>
```

Replace `<binary>` with the binary name produced by Cargo.

`defmt` logs are emitted over RTT. The default log level is configured in `.cargo/config.toml` (`DEFMT_LOG=info`). You can override it at runtime:

```bash
DEFMT_LOG=debug probe-rs run --chip STM32F303VC target/thumbv7em-none-eabihf/release/<binary>
```

## Project structure

```text
.
├── .cargo/          # target, rustflags, defmt environment
├── .vscode/         # editor / debug configurations
├── doc/             # hardware notes, Si47xx documentation, models
├── embassy/         # Embassy git submodule
├── src/
│   ├── main.rs      # application entry, hardware init, tasks
│   └── radio/
│       ├── mod.rs
│       ├── rds.rs   # RDS group decoding
│       └── si47xx.rs# Si4743 / Si47xx driver
├── svd/             # SVD files for debugging
├── Cargo.toml
├── build.rs
└── README.md
```

## Configuration

Common configuration points:

- **Chip feature:** `Cargo.toml` enables the Embassy STM32 chip feature. Change it if you use a different MCU.
- **Target:** `.cargo/config.toml` sets the default build target.
- **Defmt log level:** override with `DEFMT_LOG`, default is usually `info`.
- **REFCLK:** adjust reference clock frequency and prescaler constants in `src/main.rs`.
- **I²C address:** verify the Si4743 I²C address in `src/main.rs` (`0x63` in the current example).
- **Pins:** verify all GPIO, I²C, PWM, and EXTI assignments against your board.

## Error codes

The firmware may report errors through LED blink patterns. Example patterns:

| Blink pattern | Meaning |
|---------------|---------|
| short, short | Reset error |
| long, short | I²C CTS timeout |
| long, short, short | I²C invalid argument |
| long, long | FM no station found |

Check `src/main.rs` for the exact blink patterns used by your build.

## Troubleshooting

- **Empty or missing submodule:** run `git submodule update --init --recursive`.
- **Linker errors:** ensure `arm-none-eabi-gcc` is installed and available in `PATH`.
- **Wrong target or chip:** verify the target in `.cargo/config.toml` and the STM32 feature in `Cargo.toml`.
- **No RTT logs:** confirm the debug probe connection and use `probe-rs run` or a compatible RTT viewer.
- **I²C communication failure:** verify pull-ups, wiring, power, and the Si4743 address.
- **No OLED output:** verify the SSD1306 I²C address and shared bus configuration.
- **Radio does not seek:** check antenna, REFCLK, GPO/INT wiring, and Si4743 power/reset sequencing.

## Notes

- The `embassy` directory is a Git submodule. Re-run `git submodule update --init --recursive` after switching branches or cloning.
- Panics are reported through `panic-probe` over defmt/RTT.
- `build.rs` contains build/linker configuration; review it if you change the target or linker setup.
- This is a test project. Hardware details are examples and should be verified for your specific board.

## License

No license file is currently included in this repository. Add a license if you intend to distribute the project.

## References

- `Cargo.toml` — package and chip feature configuration.
- `.cargo/config.toml` — target, rustflags, and defmt environment.
- `src/main.rs` — application logic, hardware setup, and error codes.
- `src/radio/` — Si47xx driver and RDS decoding modules.
- `doc/` — hardware notes and Si47xx documentation.
