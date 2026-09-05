# DengFOC V4 board configuration

This file is the ESP-IDF-side board mapping for the current DengFOC V4 hardware. During migration, the legacy `arduino/include/vehicle_config.h` and `arduino/docs/硬件信息.md` remain reference sources; schematic/manual verification has priority over software assumptions.

## Confirmed GPIO map

| Function | GPIO | ADC mapping / note |
| --- | ---: | --- |
| I2C0 SDA — M0 AS5600 + BMI160 | 19 | — |
| I2C0 SCL — M0 AS5600 + BMI160 | 18 | — |
| I2C1 SDA — M1 AS5600 | 23 | — |
| I2C1 SCL — M1 AS5600 | 5 | strapping pin |
| M0 PWM A / M0_IN1 | 32 | — |
| M0 PWM B / M0_IN2 | 33 | — |
| M0 PWM C / M0_IN3 | 25 | — |
| M0 enable | 22 | — |
| M1 PWM A / M1_IN1 | 26 | — |
| M1 PWM B / M1_IN2 | 27 | — |
| M1 PWM C / M1_IN3 | 14 | JTAG pin |
| M1 enable / M_EN | 12 | strapping + JTAG pin |
| Battery voltage / VIN_MEA | 13 | ADC2_CH4 |
| M0 OUT1 current / M0_CS1 | 39 | ADC1_CH3, input-only |
| M0 OUT2 current / M0_CS2 | 36 | ADC1_CH0, input-only |
| M1 OUT1 current / M1_CS1 | 35 | ADC1_CH7, input-only |
| M1 OUT2 current / M1_CS2 | 34 | ADC1_CH6, input-only |

The two AS5600 devices both use address `0x36`, so they remain on separate I2C controllers. BMI160 uses address `0x69` and shares I2C0 with the M0 encoder.

## Schematic verification

The DengFOC V4 schematic confirms the following analog routing:

- DCBUS measurement is a fixed divider `VIN -> R12 7.5k -> VIN_MEA -> R13 1k -> GND`, with `C32 1uF` from `VIN_MEA` to ground. `VIN_MEA` is routed to ESP32 GPIO13 (`ADC2_CH4`).
- M0 uses two 10 mΩ shunts: OUT1 uses `R38` and returns as `M0_CS1`; OUT2 uses `R40` and returns as `M0_CS2`.
- M1 uses two 10 mΩ shunts: OUT1 uses `R41` and returns as `M1_CS1`; OUT2 uses `R39` and returns as `M1_CS2`.
- At the ESP32 module, these nets are routed as `M0_CS1 -> GPIO39`, `M0_CS2 -> GPIO36`, `M1_CS1 -> GPIO35`, `M1_CS2 -> GPIO34`.

## Hardware constraints

### Boot strapping

GPIO5 and GPIO12 are ESP32 strapping pins. Their external levels during reset are board-level electrical constraints and cannot be repaired by firmware after boot.

GPIO12 is especially sensitive on classic ESP32 because its reset level participates in VDD_SDIO / flash-voltage selection. Do not add or change an external pull-up on this net without checking the exact WROOM module and board schematic. Motor enable must remain electrically safe during reset.

### JTAG

Classic ESP32 normally uses GPIO12/13/14/15 for JTAG. This board currently uses GPIO12 for M1 enable, GPIO13 for battery ADC, and GPIO14 for M1 PWM C. Therefore normal motor operation and conventional JTAG debugging contend for these pins. Treat JTAG as a special debug mode rather than an always-available interface on this hardware.

### ADC and Wi-Fi

GPIO13 is `ADC2_CH4`. On classic ESP32, ADC2 cannot be relied on while Wi-Fi is active. The battery-voltage net is physically routed to GPIO13 on the V4 PCB, so changing only a firmware constant cannot move it to ADC1.

There is no spare, currently unused ADC1 GPIO on this WROOM-32 board configuration: GPIO32/33 are motor PWM outputs, and GPIO34/35/36/39 are the four current-sense inputs. GPIO37/38 are not exposed by ESP32-WROOM-32. Moving battery measurement to ADC1 therefore requires a hardware rework/redesign and would also require freeing or rerouting another function. An external ADC is another option if continuous battery measurement during Wi-Fi operation becomes necessary.

For the current firmware migration, treat battery voltage as a startup/low-rate measurement before Wi-Fi starts unless later hardware testing establishes another safe policy.

## Configuration rule

`components/board/include/board_pins.hpp` is the single software source for physical GPIO numbers in the new ESP-IDF tree. Driver components should define peripheral configuration (I2C controller number, MCPWM timer/operator, ADC channel, pull mode, interrupt mode, etc.) locally while taking the physical GPIO from the board layer.

For ADC drivers, prefer deriving ADC unit/channel from the GPIO using ESP-IDF APIs such as `adc_oneshot_io_to_channel()` instead of duplicating unit/channel constants in multiple places.
