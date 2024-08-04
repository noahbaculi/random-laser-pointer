# Random Laser Pointer

## Design Requirements

- The laser should move in a random pattern. Might be harder in a `no_std` environment.
- The laser pattern should be configurable in scale and position. Potentiometers?
- The laser will be controlled with 2 servos via PWM signals.

### Chip Comparison

I have several Arduino Nano (AVR) and ESP32 boards on hand so those are the top contenders for this project:

| Feature                         | Arduino Nano | ESP32  |
| ------------------------------- | ------------ | ------ |
| Rust Development Experience     | x            | ?      |
| `no_std` and `std` Environments | x            | ✓      |
| PWM Control Pins                | 6            | 16     |
| Analog Input Pins               | 6            | 15     |
| Size                            | Smaller      | Medium |

### Chip Selection

I will move forward with the ESP32-WROOM-32 (Xtensa architecture) boards that I have. This will allow me to evaluate the ESP32 Rust development experience with the option of a hosted `std` environment should it be necessary.

[ESP32-WROOM-32 Datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf)

#### Details

| Detail                 | Value        |
| ---------------------- | ------------ |
| Microcontroller Target | ESP32        |
| Architecture           | Xtensa       |
| Chip                   | ESP32-D0WDQ6 |

![[ESP32 WROOM Development Board Pinout.png]]

## Rust ESP32 Resources

- [The Rust on ESP Book](https://docs.esp-rs.org/book/introduction.html)

## Wiring Diagram

![Wiring Diagram](./assets/schematic.svg)
