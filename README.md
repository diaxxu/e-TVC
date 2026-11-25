## Electric Thrust Vectored Controlled Platform ##

## Overview
This project is a custom electric TVC (Thrust Vector Control) rocket and vertical landing platform. 
Instead of solid rocket motors or pyro charges, it uses a counter rotating brushless motor with a prop mounted on a dual-axis gimbal to vector thrust and balance itself in the air ( kinda like a SpaceX Falcon 9 landing, just electric and smaller ). I built this entire project solo from scratch from the airframe or gimbal or custom flight controller PCB and simulation so i can learn how real aerospace control systems and flight computers work.

## Rocket Features
- Custom modular airframe with landing legs and internal avionics bays
- Dual-axis mechanical TVC gimbal
- 100% mechanical spring parachute ejection system on top (no pyro charges or black powder)
- Custom 4-layer STM32H7 flight controller PCB designed from scratch
- Easy access to internal wiring and battery bays for quick fixes

## Design Objectives
i made it to be or have :
- Stable hover and active thrust-vector balance
- Tight mechanical gimbal with basically zero slop or binding
- 200Hz attitude and position control with GPS waypoint tracking
- High speed blackbox flight logging on an SD card
- Safe and reusable parachute recovery using a servo latch

## Technical Specifications
- Estimated Weight: ~1.2 kg 
- Max Thrust: 2.0 kg 
- Thrust-to-Weight Ratio: ~1.67
- TVC Moment Arm: 30 cm (0.30 m)
- Control Loop Speed: 200 Hz

## Propulsion & Power System
- Motor / Prop: Electric brushless setup pushing 20 N max thrust
- Battery: 6S LiPo (~22.2V)
- Regulator: MP2451DT-LF-Z step-down buck converter (for 3.3v)
- Servos: 2x TVC gimbal servos + 1x top parachute servo 
- ESC: 2x PWM ESC headers

## Avionics
I didnt want to buy an off-the-shelf FC so i routed my own PCB around an STM32H7:
- MCU: STM32H743VIT6 (Cortex-M7 @ 480 MHz)
- IMU: ICM-42688-P (low-noise 6-axis gyro)
- Baro : DPS310 (cm-level altitude precision)
- GPS : External GPS via JST connector 
- Wireless & RC: ESP32-C3 (Wi-Fi / Bluetooth) + ELRS receiver
- Storage: Micro-SD slot over SDMMC for fast blackbox logging
- Extras: USB-C, SWD debug pins, buzzer, spare ADC and USART connectors

## Firmware
The flight code runs a cascaded PID setup at 200Hz. Basically a chain of PID loops:
1. Position PID looks at target X/Y/Z vs real position and outputs target velocity
2. Velocity PID takes target velocity and outputs the target lean angle
3. Attitude PID takes target angle and tells the TVC servos how much to tilt

## Simulation
Before flashing anything to the real hardware, i built a full non-linear 6-DOF rigid-body flight simulation in MATLAB from scratch so i could test and tune the PID loops without crashing the rocket on day one. It renders the actual rocket STL in a live 3D flight viewer alongside real-time telemetry graphs (altitude, X/Y waypoint tracking, roll/pitch angles, and ESC PWM output) 
To make sure the sim wasnt just idealized fake math, i injected real hardware limitations into the physics loop:
- 10ms IMU sensor latency using a FIFO ring buffer so the controller reacts to delayed sensor data
- 0.3° servo deadband to simulate real mechanical gear slop and stiction
- Integer microsecond PWM discretization for the ESC signal (e.g. 1450us instead of smooth decimals)
- Random wind gusts, IMU gyro noise, actuator slew lag, low-pass filtered D-term, and integral anti-windup clamping

## Firmware Flashing Instructions
1. Install Required Software
2. Download and install STM32CubeProgrammer.
3. Install the STM32 USB / DFU / ST-Link drivers on your PC.
4. Connect the Flight Controller
5. Plug the STM32H7 board into your PC with a USB-C cable (or use an ST-Link on the debug pins).
6. Confirm the board shows up in device manager.
7. Enter Bootloader Mode
8. Hold the BOOT0 button while plugging in the USB-C cable to enter DFU mode.
9. Flash Firmware
10. Open STM32CubeProgrammer and connect via USB (DFU) or ST-Link.
11. Select the compiled .bin or .hex firmware file.
12. Enable Full chip erase for a clean install.
13. Click Start Programming and wait until it finishes.
14. Initial Setup
15. Power the board over USB first (no 6S LiPo yet).
16. Calibrate the ICM-42688-P accelerometer and gyro on a flat surface.
17. Check that the DPS310 baro and GPS are reading properly.
18. Set up ELRS receiver channels and hard kill-switch failsafe.
19. Verification
20. Tilt the rocket by hand and make sure the TVC gimbal moves in the opposite direction to push it back upright.
21. Test the top parachute servo trigger.
22. Do a tethered tie-down test before trying any real hover.

attention! Firmware flashing and bench testing must always be done without propellers installed and without the 6S battery plugged in unless tied down.

## Schematic ( wire diagram )



## FAQ
Q: Can this rocket really hover and land itself?
A: Yes, TWR is around 1.67 and the 200Hz cascaded PID loop actively balances pitch, roll, and altitude while GPS + baro hold position.
Q: Why use a mechanical spring for the parachute instead of pyro charges?
A: Pyro charges are messy, dangerous to test indoors, and single-use. A servo-latched spring is 100% reusable and resets in a few seconds.
Q: Why use GPS if the IMU and baro are faster?
A: The ICM-42688-P and DPS310 handle the fast 200Hz attitude and altitude loops, while GPS anchors horizontal X/Y position so it doesnt drift away during waypoint flight.
Q: What was the hardest part of building this?
A: Designing a mechanical TVC gimbal with zero play/slop and tuning the 6-DOF sim to handle 10ms IMU delay and servo deadbands without oscillating.
Q: Can I use a different flight controller?
A: Technically yes if you port the custom mixer and PID code to another H7/F7 board, though the mounting and wiring are made for my custom PCB.
Q: How safe is this to test?
A: A 6S prop spinning at full thrust is no joke. Always do tied-down static tests first, keep a kill-switch on your ELRS radio, and only test in a wide open area.

## COOL Pics




- 

## Render




## Hardest thing i faced :
- Eliminating mechanical slop and binding in the TVC gimbal ( hard )
- Tuning cascaded PIDs with 10ms IMU latency and 0.3° servo deadband without oscillation ( hard )
- Isolating the ICM-42688-P gyro from motor/prop vibrations ( moderate )
- Clean power distribution on a 6S LiPo rail ( moderate )

## BoM (secret sauce!!)
Component | Specification | Quantity | Price | Link
--- | --- | --- | --- | ---
| | | |
| | | |
| | | |

## Credits
A big thanks to Hack Club and YSWS for making this possible without them, all this project would have been just a .step file and a matlab script
also i want to thank the Reviewers who are doing an incredible job, kudos !

## Disclaimer

This project is experimental and intended for research and educational purposes. All static and flight testing must be conducted in safe and controlled environments.
