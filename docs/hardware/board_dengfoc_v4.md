# DengFOC V4 board configuration

This file is the ESP-IDF-side board mapping for the current DengFOC V4 hardware. During migration, the legacy `arduino/include/vehicle_config.h` and `arduino/docs/硬件信息.md` remain reference sources; schematic/manual verification has priority over software assumptions.

## Confirmed GPIO map

| Function | GPIO |
| --- | ---: |
| I2C0 SDA — M0 AS5600 + BMI160 | 19 |
| I2C0 SCL — M0 AS5600 + BMI160 | 18 |
| I2C1 SDA — M1 AS5600 | 23 |
| I2C1 SCL — M1 AS5600 | 5 |
| M0 PWM A | 32 |
| M0 PWM B | 33 |
| M0 PWM C | 25 |
| M0 enable | 22 |
| M1 PWM A | 26 |
| M1 PWM B | 27 |
| M1 PWM C | 14 |
| M1 enable | 12 |
| Battery voltage ADC | 13 |

The two AS5600 devices both use address `0x36`, so they remain on separate I2C controllers. BMI160 uses address `0x69` and shares I2C0 with the M0 encoder.

## Hardware constraints

### Boot strapping

GPIO5 and GPIO12 are ESP32 strapping pins. Their external levels during reset are board-level electrical constraints and cannot be repaired by firmware after boot.

GPIO12 is especially sensitive on classic ESP32 because its reset level participates in VDD_SDIO / flash-voltage selection. Do not add or change an external pull-up on this net without checking the exact WROOM module and board schematic. Motor enable must remain electrically safe during reset.

### JTAG

Classic ESP32 normally uses GPIO12/13/14/15 for JTAG. This board currently uses GPIO12 for M1 enable, GPIO13 for battery ADC, and GPIO14 for M1 PWM C. Therefore normal motor operation and conventional JTAG debugging contend for these pins. Treat JTAG as a special debug mode rather than an always-available interface on this hardware.

### ADC and Wi-Fi

GPIO13 is ADC2 on ESP32. ADC2 shares hardware resources with Wi-Fi, so continuous battery-voltage measurement while Wi-Fi telemetry is active must not be assumed reliable. If the PCB routing cannot move battery measurement to ADC1, the firmware design needs an explicit policy such as measuring before Wi-Fi starts or accepting constrained/failed reads; this must be validated on the target ESP-IDF version.

The INA240 current-sense inputs are documented on GPIO34/35/36/39, which are ADC1/input-only pins. Their exact phase-to-pin mapping should be copied into this board component only after checking the DengFOC schematic rather than guessing from the old firmware.

## Configuration rule

`components/board/include/board_pins.hpp` is the single software source for physical GPIO numbers in the new ESP-IDF tree. Driver components should define peripheral configuration (I2C controller number, MCPWM timer/operator, ADC channel, pull mode, interrupt mode, etc.) locally while taking the physical GPIO from the board layer.
