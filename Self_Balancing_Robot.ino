/*
 * Self Balancing Robot [SBR]
 * More info at: https://dryetch.blogspot.com/
 */

#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

#define RESTRICT_PITCH

// Kalman filter instances for roll and pitch.
Kalman kalmanX;
Kalman kalmanY;

// Raw IMU readings and derived angles.
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle;   // Angle integrated from gyroscope data.
double compAngleX, compAngleY;   // Angle estimated by the complementary filter.
double kalAngleX, kalAngleY;     // Angle estimated by the Kalman filter.
double corrected_x, corrected_y; // Angles corrected with the tuned offsets.

uint32_t timer;
uint8_t i2cData[14]; // Buffer for MPU-6050 register reads.

// L293D motor driver pins.
int in1_motor_left = 8;
int in2_motor_left = 7;
int in3_motor_right = 4;
int in4_motor_right = 3;

// Pulse timing used to nudge the motors.
int pwm_on = 5;  // ms on
int pwm_off = 5; // ms off

void setup() {
  // Configure the motor driver outputs.
  pinMode(in1_motor_left, OUTPUT);
  pinMode(in2_motor_left, OUTPUT);
  pinMode(in3_motor_right, OUTPUT);
  pinMode(in4_motor_right, OUTPUT);

  // Start the serial console and I2C bus.
  Serial.begin(115200);
  Wire.begin();

  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400 kHz.

  // Configure the MPU-6050 sample rate and measurement ranges.
  i2cData[0] = 7;    // 8 kHz / (7 + 1) = 1 kHz sample rate.
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling.
  i2cData[2] = 0x00; // Gyro full-scale range: +/-250 deg/s.
  i2cData[3] = 0x00; // Accelerometer full-scale range: +/-2 g.

  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once to minimize I2C transactions.
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode.

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read the WHO_AM_I register.
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Give the sensor time to stabilize.

  // Read the initial accelerometer angle and seed the filters from it.
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Convert the accelerometer reading from radians to degrees.
  #ifdef RESTRICT_PITCH
    double roll = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else
    double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  // Use the measured pose as the starting point for all angle estimates.
  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;
  timer = micros();
}

void loop() {
  // Read accelerometer, temperature, and gyroscope data in one burst.
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  // Calculate the elapsed time since the previous loop.
  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();

  #ifdef RESTRICT_PITCH
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s.
  double gyroYrate = gyroY / 131.0; // Convert to deg/s.

  #ifdef RESTRICT_PITCH
    // Handle the jump between -180 and 180 degrees before updating the filter.
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
    } else {
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
    }

    if (abs(kalAngleX) > 90) {
      gyroYrate = -gyroYrate; // Keep the gyro rate aligned with the restricted pitch model.
    }
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  #else
    // Handle the jump between -180 and 180 degrees before updating the filter.
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    } else {
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
    }

    if (abs(kalAngleY) > 90) {
      gyroXrate = -gyroXrate; // Keep the gyro rate aligned with the restricted roll model.
    }
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
  #endif

  gyroXangle += gyroXrate * dt;
  gyroYangle += gyroYrate * dt;
  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro estimate if it drifts too far away.
  if (gyroXangle < -180 || gyroXangle > 180) {
    gyroXangle = kalAngleX;
  }
  if (gyroYangle < -180 || gyroYangle > 180) {
    gyroYangle = kalAngleY;
  }

  Serial.print("\r\n");
  delay(2);

  // Apply the tuned mechanical offsets before deciding motor direction.
  corrected_x = kalAngleX - 171.746;
  corrected_y = kalAngleY - 81.80;

  // Keep the center of mass over the wheel base.
  pwm_adjust(corrected_y);
  if (corrected_y > -50 && corrected_y < -18) {
    backward();
  } else if (corrected_y >= -15 && corrected_y < 0) {
    forward();
  } else {
    stop();
  }
}

// Drive both wheels forward for one pulse.
void forward() {
  digitalWrite(in1_motor_left, HIGH);
  digitalWrite(in2_motor_left, LOW);
  digitalWrite(in3_motor_right, LOW);
  digitalWrite(in4_motor_right, HIGH);
  delay(pwm_on);

  digitalWrite(in1_motor_left, LOW);
  digitalWrite(in2_motor_left, LOW);
  digitalWrite(in3_motor_right, LOW);
  digitalWrite(in4_motor_right, LOW);
  delay(pwm_off);
}

// Drive both wheels backward for one pulse.
void backward() {
  digitalWrite(in1_motor_left, LOW);
  digitalWrite(in2_motor_left, HIGH);
  digitalWrite(in3_motor_right, HIGH);
  digitalWrite(in4_motor_right, LOW);
  delay(pwm_on);

  digitalWrite(in1_motor_left, LOW);
  digitalWrite(in2_motor_left, LOW);
  digitalWrite(in3_motor_right, LOW);
  digitalWrite(in4_motor_right, LOW);
  delay(pwm_off);
}

// Turn all motor outputs off for one pulse window.
void stop() {
  digitalWrite(in1_motor_left, LOW);
  digitalWrite(in2_motor_left, LOW);
  digitalWrite(in3_motor_right, LOW);
  digitalWrite(in4_motor_right, LOW);
  delay(pwm_on);

  digitalWrite(in1_motor_left, LOW);
  digitalWrite(in2_motor_left, LOW);
  digitalWrite(in3_motor_right, LOW);
  digitalWrite(in4_motor_right, LOW);
  delay(pwm_off);
}

// Use longer motor pulses when the tilt error is larger.
void pwm_adjust(int value_y) {
  if (value_y >= -50 && value_y <= -35) {
    pwm_on = 15; // ms on
    pwm_off = 5; // ms off
  } else if (value_y > -8 && value_y <= 0) {
    pwm_on = 15; // ms on
    pwm_off = 5; // ms off
  } else {
    pwm_on = 5;  // ms on
    pwm_off = 5; // ms off
  }
}
