# random-laser-pointer

Embedded Rust firmware for a random laser pointer station. Runs on an ESP32-C3 SuperMini driving two SG90 servos and a laser diode, with manual control from a phone over WiFi.

## Toolchain

The ESP32-C3 is RISC-V, so the stock stable toolchain works: no espup, no Xtensa channel, no `export-esp.sh`. `rust-toolchain.toml` pins stable and installs the `riscv32imc-unknown-none-elf` target and `rust-src` on first build. The only extra tool is `espflash` (`cargo install espflash`), used as the runner in `.cargo/config.toml`.

## Commands

Mirror `.github/workflows/rust_ci.yml` locally before committing:

```shell
cargo build --release
cargo fmt --all -- --check
cargo clippy --all-features --workspace -- -D warnings
```

Flash a connected board and stream serial logs:

```shell
cargo run --release
```

`cargo run` invokes `espflash flash --monitor`, which streams serial until Ctrl-C. It does not exit on its own, so an agent must not invoke it in a blocking shell. Hand the command to the human, or run it in a background shell only when a human will terminate it.

## Project todos

The README "Future work" checklist is the single source of truth for project todos. Add follow-up items there instead of scattering open checklists across docs.

## Runtime conventions

- `no_std` with an `esp-alloc` heap; `main.rs` sets up two `heap_allocator!` regions before starting the scheduler.
- Logging is `log` over `esp-println`; `esp_println::logger::init_logger_from_env()` reads `ESP_LOG`, which defaults to `info` via `.cargo/config.toml`. Override per run with `ESP_LOG=debug cargo run --release`.
- Async is Embassy on the `esp-rtos` executor. `esp_rtos::start(timg0.timer0, sw_int.software_interrupt0)` must run before spawning tasks.
- Panic handler is `esp-backtrace` with `println` output. Do not remove it.
- `main.rs` denies `clippy::mem_forget` and `clippy::large_stack_frames`. Annotate any intentional exceptions with `#[allow(..., reason = "...")]` rather than removing the lints. `.clippy.toml` sets the stack threshold; Embassy task futures live in the static task pool, so the lint measures future size, not real stack.

## Style

- Prose in this repo (README, AGENTS.md, commit messages, PR bodies) uses plain hyphens, never em or en dashes.
- Terse, command-first, reasons appended inline to non-obvious steps, no promotional adjectives. Match the register of the existing embedded repos ([turn-tracker](https://github.com/noahbaculi/turn-tracker), [desk-control-panel](https://github.com/noahbaculi/desk-control-panel), [car-sign](https://github.com/noahbaculi/car-sign)).

## References

- [ESP32-C3 SuperMini pinout](datasheets/ESP32_C3_supermini_pinout.jpg)
- [`esp-hal` examples](https://github.com/esp-rs/esp-hal/tree/main/examples)
- [The Rust on ESP Book](https://docs.esp-rs.org/book/introduction.html)
