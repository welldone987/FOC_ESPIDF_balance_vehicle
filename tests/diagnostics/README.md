# Diagnostics validation

The Python tests run on the host and validate the client decoder and wire dictionary:

```powershell
python scripts/generate_diagnostic_dictionary.py
python tests/diagnostics/test_protocol.py
```

The C++ tests compile production current_sense.cpp, motor_foc_service.cpp, PI/SVPWM,
protocol codec, remote_control and diagnostic_store with header stubs for hardware.
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
`10 Tests 0 Failures 0 Ignored` and `DIAGNOSTICS_TESTS_PASS`; stop QEMU after the marker.
These are target-emulated software tests, not native host C++ or physical ADC,
I2C, PWM, timing, or motor tests. The fake clock advances explicitly; no real sleep
or peripheral is used by the mocked drivers.

Coverage: ErrorInfo raw fields; first fault surviving ring wrap and secondary faults;
14/20-byte packets; legacy rejection; startup/fault/emergency gates; sequence and
status compatibility; replay submission failure and reconnect; ADC allocation,
channel mapping/configuration, read and conversion errors; offset mean/noise;
all four measured and both reconstructed phase trips; sampling timeout; motor
left/right driver and alignment failures; GPIO inhibition error; failed sampling
preserving accepted timestamps; encoder failure remaining latched after bus recovery.

Build and QEMU reports belong in `build/reports/diagnostics/`, not the repository root.
For example, redirect command output with `*> build/reports/diagnostics/build.log`
after creating that directory. Existing build/cache directories must not be moved
because CMake caches absolute paths.
