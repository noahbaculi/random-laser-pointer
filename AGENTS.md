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

Run the host unit tests for the `no_std` library (`settings` and the other pure modules in `src/lib.rs`). They need no board:

```shell
cargo test --target aarch64-apple-darwin --no-default-features --lib
```

`--no-default-features` drops the `firmware` feature and its `esp-*` and `embassy-*` dependencies. CI runs the same command on `x86_64-unknown-linux-gnu`.

Run the simulator: the real controller, API, and page on the Mac, no board needed:

```shell
cargo run --target aarch64-apple-darwin --no-default-features --features sim --bin sim
```

It serves the phone page at `http://<lan-ip>:8080/` (printed at startup) and a room view at `http://localhost:8080/sim` that draws the outline, laser, and target from `/status`. Settings persist to `target/sim-settings.bin` through `record`. Set `SIM_SETTINGS=<path>` to use another file. It never exits on its own, so run it in a background shell and kill it after checks. It does not cover WiFi, captive DNS, servo PWM, or flash storage.

To try the page on a real phone, join the Mac's WiFi network and open the printed LAN URL. The page is `src/page.html`, served by both the firmware and the sim.

Regenerate the README phone screenshots after changing `src/page.html`. It builds and starts its own sim on a throwaway settings file, so port 8080 must be free:

```shell
mise run screenshots
```

See [docs/screenshot-recipe.md](docs/screenshot-recipe.md) for the seeded room and sheet layout.

Check that the page's self-crossing test agrees with `geometry.rs` on the shared case table (CI runs it too, needs `node`):

```shell
node tests/page_geometry.mjs
```

Flash a connected board and stream serial logs:

```shell
cargo run --release
```

`cargo run` invokes `espflash flash --monitor --partition-table partitions.csv`, which streams serial until Ctrl-C. It does not exit on its own, so an agent must not invoke it in a blocking shell. Hand the command to the human, or run it in a background shell only when a human will terminate it.

## Project todos

The README "Future work" checklist is the single source of truth for project todos. Add follow-up items there instead of scattering open checklists across docs.

## Flash layout

`partitions.csv` is the partition table, passed to espflash by the runner in `.cargo/config.toml`. The `settings` data partition (subtype `fat`, `0x20000`) holds the `record.rs` settings record, outside the first 64 KB that every flash rewrites. Keep the app partition 64 KB aligned with headroom over the image size. Check the table and image size offline with no board:

```shell
cargo build --release
espflash save-image --chip esp32c3 --partition-table partitions.csv target/riscv32imc-unknown-none-elf/release/random-laser-pointer /tmp/image.bin
```

## Wiring diagrams

The WireViz source and the generated SVG and PNG live under `docs/wiring/`. Regenerate with `mise run wireviz`. It needs `uv` (from `mise.toml`) and `brew install graphviz` for the `dot` binary.

- WireViz models every connection as connector-cable-connector. Two cables cannot chain, and two connectors cannot connect directly.
- `->` in any string breaks Graphviz's HTML label parser. Write "to" instead.
- Validate without rendering via `uvx --from wireviz wireviz -f g -o /tmp/wv-check docs/wiring/overview.yaml`. The `g` format emits GraphViz source only and does not need `dot`.

## Runtime conventions

- `no_std` with an `esp-alloc` heap; `main.rs` sets up two `heap_allocator!` regions before starting the scheduler.
- Logging is `log` over `esp-println`; `esp_println::logger::init_logger_from_env()` reads `ESP_LOG`, which defaults to `info` via `.cargo/config.toml`. Override per run with `ESP_LOG=debug cargo run --release`.
- Async is Embassy on the `esp-rtos` executor. `esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0)` must run before spawning tasks.
- Panic handler is `esp-backtrace` with `println` output. Do not remove it.
- `main.rs` denies `clippy::mem_forget` and `clippy::large_stack_frames`. Annotate any intentional exceptions with `#[allow(..., reason = "...")]` rather than removing the lints. `.clippy.toml` sets the stack threshold; Embassy task futures live in the static task pool, so the lint measures future size, not real stack.

## Style

- Prose in this repo (README, AGENTS.md, commit messages, PR bodies) uses plain hyphens, never em or en dashes.
- Terse, command-first, reasons appended inline to non-obvious steps, no promotional adjectives. Match the register of the existing embedded repos ([turn-tracker](https://github.com/noahbaculi/turn-tracker), [desk-control-panel](https://github.com/noahbaculi/desk-control-panel), [car-sign](https://github.com/noahbaculi/car-sign)).

## References

- [ESP32-C3 SuperMini pinout](datasheets/ESP32_C3_supermini_pinout.jpg)
- [`esp-hal` examples](https://github.com/esp-rs/esp-hal/tree/main/examples)
- [The Rust on ESP Book](https://docs.esp-rs.org/book/introduction.html)
