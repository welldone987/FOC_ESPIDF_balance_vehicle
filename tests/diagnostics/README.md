# Diagnostics validation

The C++ tests compile production current_sense.cpp, motor_foc_service.cpp, bmi160_attitude.cpp, balance_controller.cpp, PI/SVPWM,
motion protocol and freshness checks and diagnostic_store with header stubs for hardware.
Only this isolated ESP-IDF test application sees those stubs. No firmware test mode
or hardware bypass is introduced into the production project.

```powershell
$env:PYTHONUTF8='1'
$env:PYTHONIOENCODING='utf-8'
& 'C:\Espressif\tools\Microsoft.v6.0.2.PowerShell_profile.ps1'
idf.py --no-ccache -C tests/diagnostics build
idf.py --no-ccache -C tests/diagnostics qemu
```

QEMU must be available in PATH; install the matching tool through the active IDF's
`tools/idf_tools.py install qemu-xtensa` if missing. Success prints
`Tests 0 Failures 0 Ignored` and `DIAGNOSTICS_TESTS_PASS`; stop QEMU after the marker.
These are target-emulated software tests, not native host C++ or physical ADC,
I2C, PWM, timing, or motor tests. The fake clock advances explicitly; no real sleep
or peripheral is used by the mocked drivers.

Coverage: ErrorInfo raw fields; first fault surviving ring wrap and secondary faults;
motion parser bounds and command age/startup boundary; ADC allocation,
channel mapping/configuration, read and conversion errors; offset mean/noise;
all four measured and both reconstructed phase trips; sampling timeout; motor
left/right driver and alignment failures; GPIO inhibition error; failed sampling
preserving accepted timestamps; encoder failure remaining latched after bus recovery.
Also covers output_age before/after PWM, the 3999/4000/4001 us encoder boundary, independent encoder
and ADC ages, no enable or PI timestamp commit on either failure, first-release
initialization, and timing snapshots retained with the first fault. Independent balance
covers zero-speed correction without valid motion, ignoring invalid targets, actual
current-path execution, command timeout, and measured tilt on the
first IMU frame (including a preceding failed read). Added current_output_age checks before/after PWM at 1999/2000/2001 us, unchanged 2ms read budgets, actual-dt complementary filtering, a 200Hz attitude/100Hz outer-loop cadence, and both BMI160 ODR register writes for 800Hz. The BMI register fake returns ready status; it does not emulate actual sensor timing or filtering. Reversed-direction regression checks both wheels and both command directions, encoder wraparound, unchanged electrical-angle coordinates, motor PI sign and vehicle-coordinate speed/Iq/Uq feedback.

Build and QEMU reports belong in `build/reports/diagnostics/`, not the repository root.
For example, redirect command output with `*> build/reports/diagnostics/build.log`
after creating that directory. Existing build/cache directories must not be moved
because CMake caches absolute paths.
