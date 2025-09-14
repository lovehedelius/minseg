#include <Wire.h>
#include <Encoder.h>

// MPU6050 register values
const uint8_t WHO_AM_I = 0x68;
const uint8_t PWR_MGMT1 = 0x6B;
const uint8_t CONFIG = 0x1A;
const uint8_t DLPF_CFG = 0;  // 0: 256 Hz, 1: 188 Hz, 2: 98 Hz, 3: 42 Hz, 4: 20 Hz, 5: 10 Hz, 6: 5 Hz
const uint8_t GYRO_CONFIG = 0x1B;
const uint8_t GYRO_XOUT_H = 0x43;
const uint8_t ACCEL_CONFIG = 0x1C;
const uint8_t ACCEL_YOUT_H = 0x3D;

// Encoder pins
const int pinA = 2;
const int pinB = 3;
Encoder encoder(pinA, pinB);

// Motor pins
const int motorDirA = 5;
const int motorDirB = 4;

// Constants
const float k[4] = {-1.0, -2.0, -1250.0, -70.0};
const float sampleTime = 0.008;
const float pi = 3.14159;
const float wheelRadius = 0.021;
const float accWeight = 0.01;
const float accBias = 0.0825;
const float maxVoltage = 3.25;
float gyroBias;
float conversionFactorGyro;
float conversionFactorAcc;

// Struct to keep track of values for each signal to be filtered
struct IIRFilter {
  float x1 = 0.0f;  // previous input
  float y1 = 0.0f;  // previous output
};

// Global variables which need to be shared across functions
float accValues[2];
float sensorValues[4];
float states[4];
float referencePosition = 0.0;
float previousEncoderRadians = 0;
unsigned long previousEncoderTime = 0;
IIRFilter accYFilter, accZFilter, encoderFilter;

void setup() {
  // Start the communication
  Serial.begin(115200);
  Wire.begin();

  // Wake up the IMU
  Wire.beginTransmission(WHO_AM_I);
  Wire.write(PWR_MGMT1);
  Wire.write(0x00);
  Wire.endTransmission();

  // Specific setup
  setupGyro();
  setupAcc();
  setupMotor();
}

void loop() {
  // Save current time
  long timeStep = millis();

  // Control the MinSeg and communicate with GUI
  readSensors();
  updateStates();
  float output = calculateOutput();
  setOutput(output);
  sendToGUI(output);
  receiveFromGUI();

  // Wait until next time step
  timeStep += sampleTime * 1000;
  long duration = timeStep - millis();
  if (duration > 0) {
    delay(duration);
  }
}

void setupGyro() {
  // Configure Digital Low-Pass Filter
  Wire.beginTransmission(WHO_AM_I);
  Wire.write(GYRO_CONFIG);
  Wire.write(DLPF_CFG);
  Wire.endTransmission();
  delay(100);  // let settings settle

  // Find conversion factor
  Wire.beginTransmission(WHO_AM_I);
  Wire.write(CONFIG);
  Wire.endTransmission(false);
  Wire.requestFrom(WHO_AM_I, (uint8_t)1);  // request 1 byte
  uint8_t gyroConfigValues = Wire.read();
  uint8_t range = (gyroConfigValues >> 3) & 0x03;  // bits 3 and 4
  switch (range) {
    case 0:
      conversionFactorGyro = 1.0f / 131.0f * pi / 180.0f;
      break;
    case 1:
      conversionFactorGyro = 1.0f / 65.5f * pi / 180.0f;
      break;
    case 2:
      conversionFactorGyro = 1.0f / 32.8f * pi / 180.0f;
      break;
    case 3:
      conversionFactorGyro = 1.0f / 16.4f * pi / 180.0f;
      break;
  }

  // Calibrate gyro bias, MinSeg must be lying still during this
  long biasSum = 0;
  int numberOfSamples = 1000;
  for (int i = 0; i < numberOfSamples; i++) {
    Wire.beginTransmission(WHO_AM_I);
    Wire.write(GYRO_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(WHO_AM_I, 2, true);

    int16_t xRaw = (Wire.read() << 8) | Wire.read();
    biasSum += xRaw;
  }
  gyroBias = biasSum / numberOfSamples;
}

void setupAcc() {
  // Configure Digital Low-Pass Filter
  Wire.beginTransmission(WHO_AM_I);
  Wire.write(ACCEL_CONFIG);
  Wire.write(DLPF_CFG);
  Wire.endTransmission();
  delay(100);  // let settings settle

  // Find conversion factor
  Wire.beginTransmission(WHO_AM_I);
  Wire.write(CONFIG);
  Wire.endTransmission(false);
  Wire.requestFrom(WHO_AM_I, (uint8_t)1);  // request 1 byte
  uint8_t accelConfigValues = Wire.read();
  uint8_t range = (accelConfigValues >> 3) & 0x03;  // bits 3 and 4
  switch (range) {
    case 0:
      conversionFactorAcc = 9.81567f / 16384.0f;
      break;
    case 1:
      conversionFactorAcc = 9.81567f / 8192.0f;
      break;
    case 2:
      conversionFactorAcc = 9.81567f / 4096.0f;
      break;
    case 3:
      conversionFactorAcc = 9.81567f / 2048.0f;
      break;
  }
}

void setupMotor() {
  // Set up the motor and make sure it's turned off
  pinMode(motorDirA, OUTPUT);
  pinMode(motorDirB, OUTPUT);
  analogWrite(motorDirA, 0);
  analogWrite(motorDirB, 0);
}

void readSensors() {
  // Delegate to functions for each sensor
  float gyroValue = readGyro();
  readAcc();
  float encoderValue = readEncoder();

  // Collect values in one array
  sensorValues[0] = gyroValue;
  sensorValues[1] = accValues[0];
  sensorValues[2] = accValues[1];
  sensorValues[3] = encoderValue;
}

float readGyro() {
  // Read 2 bytes starting at the GYRO_XOUT_H register
  Wire.beginTransmission(WHO_AM_I);
  Wire.write(GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(WHO_AM_I, 2, true);

  // Convert to rad/s, taking bias into account
  int16_t xRaw = (Wire.read() << 8) | Wire.read();
  float xGyro = (xRaw - gyroBias) * conversionFactorGyro;
  return -xGyro;
}

void readAcc() {
  // Read 4 bytes starting at the ACCEL_YOUT_H register
  Wire.beginTransmission(WHO_AM_I);
  Wire.write(ACCEL_YOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(WHO_AM_I, 4, true);

  // Convert to m/s^2
  int16_t yRaw = (Wire.read() << 8) | Wire.read();
  int16_t zRaw = (Wire.read() << 8) | Wire.read();
  accValues[0] = yRaw * conversionFactorAcc * -1;  // flip so positive direction is upwards
  accValues[1] = zRaw * conversionFactorAcc;
}

float readEncoder() {
  // Read encoder value and convert to radians
  long pulses = encoder.read();
  float radians = pulses * 2 * pi / 720;
  return radians;
}

void updateStates() {
  // Unpack values to local variables
  float gyroValue = sensorValues[0];
  float accValueY = sensorValues[1];
  float accValueZ = sensorValues[2];
  float encoderValue = sensorValues[3];

  // Use LPF, complementary filter and derivative functions to calculate needed quantities
  float filteredAccY = lowPassFilter(accValueY, 0.2f, accYFilter);
  float filteredAccZ = lowPassFilter(accValueZ, 0.2f, accZFilter);
  float tiltAngle = calculateTiltAngle(gyroValue, filteredAccY, filteredAccZ);
  float encoderVelocity = calculateEncoderVelocity(encoderValue);
  float filteredEncoderVelocity = lowPassFilter(encoderVelocity, 0.4f, encoderFilter);

  // Calculate states
  float x = (tiltAngle + encoderValue) * wheelRadius;
  float xDot = (gyroValue + filteredEncoderVelocity) * wheelRadius;
  float alpha = tiltAngle;
  float alphaDot = gyroValue;

  // Save in the states array
  states[0] = x - referencePosition;
  states[1] = xDot;
  states[2] = alpha;
  states[3] = alphaDot;
}

float calculateTiltAngle(float gyroValue, float accY, float accZ) {
  // Initialize values for first iteration
  static float angle = 0.0f;
  static unsigned long previousTime = millis();

  // Calculate elapsed time
  unsigned long currentTime = millis();
  float deltaSeconds = (currentTime - previousTime) / 1000.0f;
  previousTime = currentTime;

  // Complementary filter to combine values from accelerometer and gyrsocope
  float accAngle = atan2(accZ, accY) - accBias;
  float deltaRadians = -gyroValue * deltaSeconds;
  angle = (1 - accWeight) * (angle + deltaRadians) + accWeight * accAngle;
  return -angle;
}

float calculateEncoderVelocity(float currentRadians) {
  // Calculate elpased time and change in wheel angle
  unsigned long currentTime = millis();
  float deltaSeconds = (currentTime - previousEncoderTime) / 1000.0f;
  float deltaRadians = currentRadians - previousEncoderRadians;
  previousEncoderRadians = currentRadians;
  previousEncoderTime = currentTime;

  // Approximate the time derivative of the wheel angle
  float velocity = deltaRadians / deltaSeconds;
  return velocity;
}

float calculateOutput() {
  // Calculate control signal u = -Kx
  float output = states[0] * k[0] + states[1] * k[1] + states[2] * k[2] + states[3] * k[3];
  return output;
}

void setOutput(float output) {
  // Determine direction and calculate PWM
  int direction = (output > 0) ? motorDirA : motorDirB;
  int otherDirection = (output > 0) ? motorDirB : motorDirA;
  float pwm = min(fabs(output) / maxVoltage * 255.0f, 255.0f);

  // Send the signal to the motor
  analogWrite(otherDirection, 0);
  analogWrite(direction, (int)pwm);
}

void sendToGUI(float voltage) {
  // Send the plot values to the GUI
  float cappedVoltage = max(min(voltage, maxVoltage), -maxVoltage);
  Serial.print("S"); // S signals start of message line
  Serial.print(cappedVoltage, 4);
  Serial.print(",");
  Serial.print(states[2], 4);
  Serial.print(",");
  Serial.print(states[3], 4); 
  Serial.print(",");
  Serial.print(states[0] + referencePosition, 4); // To get the actual position and not the state
  Serial.print(",");
  Serial.print(states[1], 4);
  Serial.print(",");
  Serial.println(referencePosition, 4);
}

void receiveFromGUI() {
  // Read messages from the GUI and update reference variable accordingly
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.length() > 0) {
      float value = line.toFloat();
      referencePosition = value;
    }
  }
}

float lowPassFilter(float input, float timeConstant, IIRFilter& state) {
  // Calculate feedforward and feedback coefficients
  float b1 = sampleTime / timeConstant;
  float a1 = sampleTime / timeConstant - 1;

  // Calculate output and save in struct
  float output = b1 * state.x1 - a1 * state.y1;
  state.x1 = input;
  state.y1 = output;
  return output;
}