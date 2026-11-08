# Wiring diagrams

WireViz source for the station wiring. `overview.yaml` covers the panel USB-C through the power switch to the ESP32-C3 SuperMini, the laser diode on GPIO5, and the servos on GPIO3 (X) and GPIO2 (Y).

The USB lead plugs into the SuperMini's onboard USB port, so its link to `U_ESP` is not drawn.

| Color  | Net                                         |
| ------ | ------------------------------------------- |
| RD     | +V (5V on the USB side, 3V3 to the servos and laser) |
| BK, BN | GND (BN on the servo leads to match SG90 wire colors) |
| OG     | Servo PWM                                   |
| WH     | USB D-                                      |
| GN     | USB D+                                      |

Designators: `J_*` connectors, `U_*` modules, `M_*` motors, `S_*` switches, `W_*` cables.

## Regenerate

```shell
mise run wireviz
```

Setup and gotchas are in [`AGENTS.md`](../../AGENTS.md) under "Wiring diagrams".
