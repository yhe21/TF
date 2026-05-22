# TF
Arduino_1 Program Description
1. Purpose

This Arduino_1 program controls the tray positioning system and coordinates signals between the VT6 robot and the tray motors. It manages six part tray motors, two nameplate tray motors, plate selection signals, second-place positioning, eject requests, and system-ready feedback.

The program is responsible for moving the selected tray motor to the correct working position while keeping all other tray motors in standby positions. It also verifies robot feedback signals and prevents incorrect tray movement through communication and position checks.

2. Controlled Devices

Arduino_1 controls the following devices:

Motor 1–6: part tray motors
Motor 7–8: nameplate tray motors
Plate selection output signals
Second place output signal
Ejecting output signal
Nameplate tray status outputs
Push buttons and limit switches for manual operation and homing verification
3. Main Inputs

The main inputs to Arduino_1 include:

Part tray buttons, used for manual tray return or reset
Nameplate tray buttons
Limit switches for part tray and nameplate tray homing checks
Plate echo confirmation signals from VT6
Second place request from VT6
Plate eject request from VT6
Tray eject requests from T6

Robot-related input signals are treated as high-level active. Buttons and mechanical sensors are treated as low-level active.

4. Main Outputs

The main outputs from Arduino_1 include:

Fixture OK / at-position signal
Plate selection bit signals
Second place ON signal
Plate ejecting signal
Nameplate tray 1 and tray 2 availability signals

These outputs are sent to VT6, T6, or external relay/optocoupler boards.

5. Motor State Logic

Each part tray motor uses a state-machine structure. The main tray motor states include:

Station 1 unsent
Station 1 sent
Station 1 acknowledged
Station 1 reached
Station 2 unsent
Station 2 sent
Station 2 acknowledged
Station 2 reached
Station 3 unsent
Station 3 sent
Station 3 acknowledged
Station 3 reached

Station 1 is the return or reset position.
Station 2 is the middle standby position.
Station 3 is the working position.

Only the motor selected by the plate variable is allowed to move to Station 3. Other motors remain at Station 2 unless manually returned.

6. Plate Selection Logic

The program continuously checks whether any motor is currently in the Station 3 process. If a motor is already in Station 3, the plate selection is locked and cannot change.

When no motor is in Station 3, the program searches for the next available motor starting from the next motor after the current plate. A motor is considered available when it has reached Station 2.

Once a new plate is selected, Arduino_1 outputs the plate number using three digital output bits.

7. Second Place Logic

Station 3 has two possible working positions:

First position
Second position

The target is selected based on the second place request signal from VT6.

If the motor is already at Station 3 and the second place request changes, the motor returns to the Station 3 unsent state and moves to the new requested position.

8. Eject Logic

When VT6 sends a plate eject request, Arduino_1 turns off the OK signal, turns on the ejecting signal, waits for the eject request to clear, and then turns off the ejecting signal.

Manual eject can also be triggered by holding the corresponding motor button.

9. Homing Logic

The program performs homing for all tray motors during startup. It checks the homing status flag from each motor, confirms successful homing, and verifies mechanical limit switch behavior.

The homing process includes:

Checking current homing status
Triggering collision homing or closest single-turn homing
Waiting for homing completion
Moving motors to verification positions
Checking limit switch states
Moving motors to their final starting positions

If any motor fails homing or a limit switch does not respond as expected, the program stops for safety.

10. Communication and Position Verification

The program does not rely on active motor replies for movement completion. Instead, it actively queries the motor:

S_TPOS is used to confirm that the target position was correctly received.
S_CPOS is used to confirm that the motor reached the target position.
S_OFLAG is used to check homing status.
S_SFLAG may be used during startup to check motor status and confirm motor enable.

This reduces the risk of missed serial feedback and improves reliability.
