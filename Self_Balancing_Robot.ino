/**
* Code for for the Self Balancing Robot [SBR]
* More info at: dryetch.blogspot.com
*
*/
#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#define RESTRICT_PITCH 

/**
* Create the Kalman instances
*
*/
Kalman kalmanX;
Kalman kalmanY;

/**
* IMU Data
* 
*/
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Gyroscope angle
double compAngleX, compAngleY; // Complementary filter angle 
double kalAngleX, kalAngleY; // Angle after Kalman filter
double corrected_x, corrected_y; // Corrected with offset

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

/**
* Motor Variables
*
*/
int in1_motor_left = 8;
int in2_motor_left = 7;
int in3_motor_right = 4;
int in4_motor_right = 3;

/**
* PWM timings
*
*/
int pwm_on = 5; // ms ON
int pwm_off = 5; // ms OFF

//------------------------------------------------------------------------------
void setup() {
  // Define outputs 
  pinMode(in1_motor_left, OUTPUT);
  pinMode(in2_motor_left, OUTPUT);
  pinMode(in3_motor_right, OUTPUT);
  pinMode(in4_motor_right, OUTPUT);
  // Start serial console
  Serial.begin(115200);
  // Initiate the Wire library and join the I2C bus as a master or slave
  Wire.begin();

  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g

  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }
  delay(100); // Wait for sensor to stabilize

/** 
* Set kalman and gyro starting angle
*
*/
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  #ifdef RESTRICT_PITCH
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

/** 
* Set starting angle
*
*/
  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;
  timer = micros();
}

//------------------------------------------------------------------------------
void loop() {
/** 
* Update all the values
*
*/
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];
  // Calculate delta time
  double dt = (double)(micros() - timer) / 1000000; 
  timer = micros();


  #ifdef RESTRICT_PITCH
    double roll  = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  #else 
    double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  #endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

  #ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
      kalmanX.setAngle(roll);
      compAngleX = roll;
      kalAngleX = roll;
      gyroXangle = roll;
    } else
      kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
      gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  #else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
      kalmanY.setAngle(pitch);
      compAngleY = pitch;
      kalAngleY = pitch;
      gyroYangle = pitch;
    } else
      kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90)
      gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  #endif

    gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

    // Reset the gyro angle when it has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
      gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
      gyroYangle = kalAngleY;

    Serial.print("\r\n");
    delay(2);
    // Corrected angles with offset
    corrected_x=kalAngleX-171,746;
    corrected_y=kalAngleY-81,80;

/** 
* Set the vertical mass center
*
*/
  pwm_adjust(corrected_y);
  if(corrected_y>-50 && corrected_y<-18){
    backward();    
  }else if(corrected_y>=-15 && corrected_y<0){
    forward();          
  }else{
    stop();
  }
}

//------------------------------------------------------------------------------
/** 
* Wheel movement funcions
*
*/
void forward(){
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

void backward(){
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

void stop(){
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

void pwm_adjust(int value_y){
  if(value_y >= -50 && value_y <= -35){
    pwm_on = 15; // ms ON
    pwm_off = 5; // ms OFF  
  }else if(value_y > -8  && value_y <= 0) {
    pwm_on = 15; // ms ON
    pwm_off = 5; // ms OFF
  }else{
    pwm_on = 5; // ms ON
    pwm_off = 5; // ms OFF
  }  
}
