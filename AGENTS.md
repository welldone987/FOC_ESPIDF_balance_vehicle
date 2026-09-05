# Project

Pure ESP-IDF firmware for a DengFOC V4 based ESP32 balancing vehicle.

Target environment:
- ESP32-WROOM-32 / classic ESP32
- ESP-IDF v6.0.2
- C++ is the default application language

# Build and validation

Use the ESP-IDF toolchain only.

Local build:
`eim run "idf.py build"`

CI build:
`idf.py build`

After changing C/C++ source, CMake, Kconfig, sdkconfig defaults, or component dependencies, run a full project build before claiming completion. Fix errors introduced by the change. Do not invoke GCC, CMake, Ninja, or esptool directly unless diagnosing the build system itself.

# Migration policy

`arduino/` is a read-only reference implementation during migration unless the user explicitly asks to modify it.

Migrate one subsystem at a time. Preserve existing control behavior, units, signs, limits, protocol formats, and timing assumptions unless a change is explicitly requested. Do not combine framework migration with controller retuning.

Do not add Arduino framework dependencies to the new ESP-IDF application. SimpleFOC in `arduino/` is a behavioral reference only; its replacement must remain behind the motor service boundary.

# Board configuration

All physical DengFOC V4 GPIO assignments belong in `components/board/include/board_pins.hpp`.

Application and driver modules must use the board definitions instead of hard-coded GPIO numbers. Do not change a board pin mapping without checking `docs/hardware/board_dengfoc_v4.md` and the board schematic/manual.

Treat boot strapping pins, JTAG pins, ADC unit conflicts, motor enable lines, and PWM outputs as hardware constraints rather than ordinary software configuration.

# Safety

Do not automatically flash hardware, erase flash, burn eFuses, or enable motors unless explicitly requested.

Motor outputs are safety-critical. A successful build is not permission to energize the inverter or run FOC alignment.

# Architecture

Prefer ESP-IDF components for independent subsystems. Keep hardware access separated from control algorithms and network services.

Expected module boundaries include:
- board
- common/config
- imu
- encoder
- motor
- control
- telemetry
- ble

Avoid blocking network, filesystem, logging, or dynamic allocation operations in high-frequency control paths.
