# Homing calibration regression tests

The host test includes the production sketch and substitutes Arduino I/O with a
fake serial bus. It does not communicate with hardware.

From a Visual Studio Developer Command Prompt, in the repository root:

```bat
cl /nologo /EHsc /std:c++14 /utf-8 tests\homing_calibration\test.cpp /Fe:"%TEMP%\TF_homing_calibration_test.exe" /Fo:"%TEMP%\TF_homing_calibration_test.obj"
"%TEMP%\TF_homing_calibration_test.exe"
```

Coverage includes all eight motor addresses, positive/negative/zero offsets,
degree-to-wire conversion, and blocking premature resets when target, position,
status, or reply validation fails. A separate case checks failed zero readback.
Motion and reset commands produce no automatic replies, matching Response=None.

This tests calibration logic, not physical motion, the complete startup sequence,
or installation-specific offsets. Hardware verification is still required after
configuring the production offset array.
