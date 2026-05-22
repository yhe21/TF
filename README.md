# Arduino_2 Program Description

## 1. Purpose

Arduino_2 controls the stamping and glue fixture station. It receives process requests from the T6 robot and controls the fixture positioning motor, glue solenoid, stamping solenoid, and fixture OK signal.

The station currently uses a **dual-head glue setup**, so the old glue shutter function has been removed from the active process. The fixture only needs to stop at one glue position for glue application.

---

## 2. Main Functions

Arduino_2 is responsible for:

- Moving the fixture between OK, stamp, and glue positions
- Performing the stamp-only cycle when requested by T6
- Performing the stamp-and-glue cycle when requested by T6
- Activating the glue solenoid during the glue process
- Activating the stamping solenoid during the stamping process
- Checking the stamp sensor during the stamping stroke
- Reporting fixture OK status back to T6
- Performing startup motor enable, collision homing, and position verification
- Entering a fatal error state if a critical fault is detected

---

## 3. Controlled Devices

Arduino_2 controls one active positioning motor:

| Device | Function |
|---|---|
| Motor 9 | Moves the fixture between OK, stamp, and glue positions |
| Glue solenoid | Dispenses glue through the dual-head glue system |
| Stamp solenoid | Drives the stamping cylinder |
| Fixture OK output | Signals T6 that the fixture is back at the OK position |

The previous Motor 10 shutter function is no longer part of the active process. Any remaining shutter-related code should be treated as legacy code unless it is reactivated later.

---

## 4. Inputs

| Arduino Pin | Signal | Description |
|---|---|---|
| D44 | Stamp + Glue Request | Sent by T6 to start the full stamp-and-glue cycle |
| D45 | Stamp Only Request | Sent by T6 to start the stamp-only cycle |
| D46 | Limit Switch | Used during homing and OK position verification |
| D47 | Stamp Sensor | Used to confirm the stamping cylinder stroke and recovery |
| D30 | Purge Switch | Reserved for manual glue purge if enabled |

Robot request signals are high-level active. Mechanical sensors should follow the actual wiring logic defined during machine setup.

---

## 5. Outputs

| Arduino Pin | Signal | Description |
|---|---|---|
| D48 | Fixture at OK | Sent to T6 when the fixture is at the OK position and ready |
| D49 | Glue Solenoid | Turns glue dispensing on and off |
| D50 | Stamp Solenoid | Turns the stamping cylinder on and off |

---

## 6. Fixture Positions

Motor 9 uses the following main positions:

| Position | Purpose |
|---|---|
| OK Position | Safe / ready position for T6 interaction |
| Stamp Position | Position where the stamping cylinder presses the part |
| Glue Position | Position where the dual-head glue system applies glue |

Since the glue system now uses two glue heads, only one glue stop is required in the active cycle.

---

## 7. Stamp + Glue Cycle

When T6 sends a Stamp + Glue request, Arduino_2 performs the following sequence:

1. Turns off the Fixture OK signal.
2. Moves the fixture to the stamp position.
3. Activates the stamping solenoid.
4. Waits for the stamp sensor to confirm the stroke.
5. Turns off the stamping solenoid.
6. Confirms the stamp sensor has recovered.
7. Moves the fixture to the glue position.
8. Turns on the glue solenoid for the defined glue time.
9. Turns off the glue solenoid.
10. Moves the fixture back to the OK position.
11. Turns on the Fixture OK signal.

---

## 8. Stamp Only Cycle

When T6 sends a Stamp Only request, Arduino_2 performs the following sequence:

1. Turns off the Fixture OK signal.
2. Moves the fixture to the stamp position.
3. Activates the stamping solenoid.
4. Waits for the stamp sensor to confirm the stroke.
5. Turns off the stamping solenoid.
6. Confirms the stamp sensor has recovered.
7. Moves the fixture back to the OK position.
8. Turns on the Fixture OK signal.

No glue operation is performed during this cycle.

---

## 9. Homing and Startup Logic

During startup, Arduino_2 performs a motor preparation and homing sequence before normal operation.

The startup sequence includes:

1. Initializing serial communication and I/O.
2. Checking the stamp sensor condition.
3. Reading the motor status flag.
4. Waiting for Motor 9 to respond if it is not online.
5. Enabling Motor 9.
6. Verifying that Motor 9 is enabled.
7. Performing collision homing.
8. Performing the normal homing routine.
9. Verifying the limit switch state.
10. Moving the fixture to the OK position.

After homing, the limit switch behavior is checked to confirm that the fixture has moved away from the homing position and reached the correct OK position.

---

## 10. Motion Verification

Every commanded fixture movement is verified in two steps:

### 10.1 Target Confirmation

Arduino_2 reads the motor target position to confirm that the motor received the commanded target.

### 10.2 Arrival Confirmation

Arduino_2 reads the motor current position to confirm that the fixture reached the target position.

If the target is not confirmed, the command is resent within the timeout window. If the motor does not reach the commanded position within the allowed time, the program enters a fatal error state.

---

## 11. Error Handling

Arduino_2 uses a fatal error function for critical failures. When a fatal error occurs, the program prints the error message to the serial monitor and enters an infinite loop.

Typical fatal error conditions include:

- Motor not responding
- Motor enable failure
- Homing failure
- Limit switch state mismatch
- Stamp sensor not triggered
- Stamp sensor not recovered
- Motor movement timeout

This prevents the station from continuing after an unsafe or unknown condition.

---

## 12. Notes for Current Version

The current version is based on the updated dual-head glue design. The old glue shutter logic should not be described as part of the active machine cycle.

Recommended cleanup items:

- Remove or clearly mark old Motor 10 shutter code as legacy.
- Rename the active glue position to a general “Glue Position” instead of “Glue1” or “Glue2” if only one glue stop is used.
- Keep all adjustable positions and timing values near the top of the Arduino file for easier tuning during commissioning.
