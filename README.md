# Self Balancing Robot

This project demonstrates a two-wheel self-balancing robot built around an Arduino Nano and an MPU-6050 IMU. The sketch estimates the robot pitch with a Kalman filter and drives the motors in short pulses to keep the chassis upright.

Project notes and build photos were originally published at <https://dryetch.blogspot.com/>.

## Bill of Materials

1. Arduino Nano V3.0 ATmega328P 5 V board
2. MPU-6050 3-axis accelerometer and gyroscope module
3. Two DC gear motors with wheels
4. Rhino 610 mAh 2S 7.4 V 20C LiPo battery
5. HobbyKing E4 battery charger
6. L293D motor driver - Quadruple Half-H Driver
7. 33 x 70 mm single-sided PCB
8. 100 x 90 x 4.74 mm acrylic chassis
9. Screws, spacers, and other mounting hardware

## Repository Layout

- `Self_Balancing_Robot.ino`: main robot control loop, IMU setup, and motor control
- `I2C.ino`: helper routines for MPU-6050 I2C communication
- `Kalman.h`: Kalman filter implementation from TKJ Electronics

## Wiring

The sketch currently uses these Arduino pins for the L293D motor driver:

- `D8`: left motor forward
- `D7`: left motor backward
- `D4`: right motor forward
- `D3`: right motor backward

The MPU-6050 connects over I2C:

- `A4`: SDA
- `A5`: SCL
- `5V`: VCC
- `GND`: GND

Double-check motor polarity before final assembly. If the robot moves in the wrong direction when it leans, swap the motor wires or adjust the direction helpers in the sketch.

## Software Requirements

- Arduino IDE or Arduino CLI with support for `arduino:avr:nano`
- The standard Arduino `Wire` library

No additional third-party Arduino library is required because the Kalman filter implementation is included in this repository.

## Uploading the Sketch

1. Open `Self_Balancing_Robot.ino` in the Arduino IDE.
2. Select the correct board and processor for your Nano.
3. Connect the Nano by USB.
4. Upload the sketch.
5. Open the serial monitor at `115200` baud if you want to see startup errors.

## Calibration Notes

The current control loop uses a fixed pitch offset:

- `kPitchOffsetDeg = 81.80`

That value is used to convert the raw filtered pitch into the robot's balance error. In practice, you will likely need to retune it for your own frame, wheel diameter, battery position, and sensor mounting angle.

A simple way to recalibrate:

1. Hold the robot at the mechanical position you consider "balanced".
2. Print `kalAngleY` to the serial monitor temporarily.
3. Set `kPitchOffsetDeg` to that measured value.
4. Test with the wheels off the ground first.

The drive thresholds and pulse timings are also empirical and may need adjustment:

- `kBackwardDriveMinDeg`
- `kBackwardDriveMaxDeg`
- `kForwardDriveMinDeg`
- `kForwardDriveMaxDeg`
- `kDefaultPwmOnMs`
- `kBoostPwmOnMs`

## License

This repository includes GPLv2-licensed code from TKJ Electronics in `Kalman.h` and `I2C.ino`. See the [LICENSE](LICENSE) file for the full license text.
