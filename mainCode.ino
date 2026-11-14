
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

float targetAngle = 0.0;
float angle = 0.0;
float error = 0.0;
float pidOutput = 0.0;
float motorSpeed = 0.0;

const int PWMA = 5;
const int AIN1 = 7;
const int AIN2 = 8;

const int PWMB = 6;
const int BIN1 = 9;
const int BIN2 = 10;

const int STBY = 4;

float Kp = 8.0;
float Ki = 0.0;
float Kd = 0.2;

float integral = 0.0;
float previousError = 0.0;
float gxBias = 0.0;
float angleOffset = 0.0;

unsigned long previousTime = 0;
unsigned long previousIMUTime = 0;

const unsigned long loopInterval = 10;
unsigned long lastLoopTime = 0;

float safetyAngle = 40.0;

void stopMotors() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

bool mpuResponds(byte &who) {
  Wire.beginTransmission(0x68);
  Wire.write(0x75);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom((uint8_t)0x68, (uint8_t)1) != 1) {
    return false;
  }

  who = Wire.read();
  return true;
}

float readAccelAngle() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  return atan2((float)ay, (float)az) * 180.0 / PI;
}

bool calibrateIMU() {
  const int N = 500;
  long sumGX = 0;
  float sumAngle = 0.0;

  Serial.println("Keep robot upright and completely still...");
  delay(2000);

  for (int i = 0; i < N; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    sumGX += gx;
    sumAngle += atan2((float)ay, (float)az) * 180.0 / PI;

    delay(5);
  }

  gxBias = (float)sumGX / N;
  angleOffset = sumAngle / N;

  Serial.print("GX bias: ");
  Serial.println(gxBias, 2);

  Serial.print("Angle offset: ");
  Serial.println(angleOffset, 2);

  angle = 0.0;
  previousIMUTime = 0;

  return true;
}

float getAngle() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accelAngle =
      atan2((float)ay, (float)az) * 180.0 / PI
      - angleOffset;

  unsigned long currentIMUTime = micros();

  if (previousIMUTime == 0) {
    previousIMUTime = currentIMUTime;
    angle = accelAngle;
    return angle;
  }

  float dt =
      (currentIMUTime - previousIMUTime) / 1000000.0;

  previousIMUTime = currentIMUTime;

  if (dt <= 0.0 || dt > 0.1) {
    return angle;
  }

  float gyroRate = (gx - gxBias) / 131.0;

  angle =
      0.98 * (angle + gyroRate * dt)
      + 0.02 * accelAngle;

  return angle;
}

float calculatePID(float currentError) {
  unsigned long currentTime = millis();

  float dt = (currentTime - previousTime) / 1000.0;

  if (dt <= 0.0 || dt > 0.1) {
    previousTime = currentTime;
    previousError = currentError;
    return 0.0;
  }

  float P = Kp * currentError;

  integral += currentError * dt;
  integral = constrain(integral, -100.0, 100.0);

  float I = Ki * integral;

  float derivative =
      (currentError - previousError) / dt;

  float D = Kd * derivative;

  float output = P + I + D;
  output = constrain(output, -255.0, 255.0);

  previousTime = currentTime;
  previousError = currentError;

  return output;
}

void moveMotors(float speed) {
  speed = constrain(speed, -255.0, 255.0);

  int pwm = (int)abs(speed);

  if (speed > 0.0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else if (speed < 0.0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {
    stopMotors();
    return;
  }

  analogWrite(PWMA, pwm);
  analogWrite(PWMB, pwm);
}

void setup() {
  Serial.begin(115200);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  stopMotors();

  Wire.begin();
  Wire.setClock(100000);

  mpu.initialize();

  byte who = 0xFF;

  if (!mpuResponds(who)) {
    Serial.println("MPU not responding!");
    stopMotors();
    digitalWrite(STBY, LOW);

    while (true) {
      delay(100);
    }
  }

  Serial.print("MPU responded, WHO_AM_I: 0x");
  Serial.println(who, HEX);

  calibrateIMU();

  previousTime = millis();
  lastLoopTime = millis();
}

void loop() {
  unsigned long now = millis();

  if (now - lastLoopTime < loopInterval) {
    return;
  }

  lastLoopTime = now;

  angle = getAngle();
  error = targetAngle - angle;

  if (abs(angle - targetAngle) > safetyAngle) {
    stopMotors();
    integral = 0.0;
    previousError = error;
    previousTime = millis();
    return;
  }

  pidOutput = calculatePID(error);
  motorSpeed = pidOutput;

  moveMotors(motorSpeed);

  Serial.print("Angle: ");
  Serial.print(angle, 2);

  Serial.print(" | Error: ");
  Serial.print(error, 2);

  Serial.print(" | PID: ");
  Serial.print(pidOutput, 2);

  Serial.print(" | Motor Speed: ");
  Serial.println(motorSpeed, 2);
}
