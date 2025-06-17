#include <BluetoothSerial.h>
#include <ESP32Servo.h>
#include <IRremote.h>

// Bluetooth
BluetoothSerial SerialBT;
const String bluetoothName = "KV-1";

// Motor 1
const int ENA = 19;
const int IN1 = 18;
const int IN2 = 21;

// Motor 2
const int ENB = 4;
const int IN3 = 17;
const int IN4 = 16;

// Turret Servo
Servo TurretServo;
const int turretServoPin = 32;
const int defaultTurretAngle = 90;
const int traverseStep = 10;
int turretAngle = defaultTurretAngle;
int targetTurretAngle = turretAngle;

// IR Transmitter
const int transmitterPin = 25;

// Firing
const unsigned long firingCooldown = 500; 
unsigned long lastFired = 0;

// PWM
const int freq = 1000;
const int resolution = 8;

// Speed
int minimumThreshold = 32;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcAttach(ENA, freq, resolution);
  ledcAttach(ENB, freq, resolution);

  ESP32PWM::allocateTimer(3);
  TurretServo.attach(turretServoPin);

  IrSender.begin(transmitterPin);

  Serial.begin(115200);
  SerialBT.begin(bluetoothName);

  TurretServo.write(0);
  delay(1000);
  TurretServo.write(180);
  delay(1000);
  TurretServo.write(defaultTurretAngle);
}

void loop() {
  if (turretAngle != targetTurretAngle) {
    if (turretAngle < targetTurretAngle) turretAngle++;
    else if (turretAngle > targetTurretAngle) turretAngle--;

    TurretServo.write(turretAngle);
  }

  if (SerialBT.available()) {
    String input = SerialBT.readStringUntil('\n');

    int comma1 = input.indexOf(',');
    int comma2 = input.indexOf(',', comma1 + 1);

    float distance = input.substring(0, comma1).toFloat();
    float angle = input.substring(comma1 + 1, comma2).toFloat();
    int turretLeft = input.substring(comma2 + 1, comma2 + 2).toInt();
    int turretRight = input.substring(comma2 + 2, comma2 + 3).toInt();
    int fire = input.substring(comma2 + 3).toInt();

    if (distance != 0) {
      int speed = distance * -255;
      float ratio = abs(cos(angle * 3.14 / 180));

      int leftSpeed, rightSpeed;

      if (angle > 0 && angle < 180) {
        leftSpeed = speed;
        rightSpeed = speed * ratio;
      } else {
        rightSpeed = speed;
        leftSpeed = speed * ratio;
      }

      if (angle > 90 && angle < 270) {
        leftSpeed *= -1;
        rightSpeed *= -1; 
      }

      if (abs(leftSpeed) < minimumThreshold) leftSpeed = 0;
      if (abs(rightSpeed) < minimumThreshold) rightSpeed = 0;

      setMotorSpeed(leftSpeed, rightSpeed);
    }

    if (turretLeft == 1 || turretRight == 1) {
      if (turretLeft == 1) {
        targetTurretAngle += traverseStep;
      }

      if (turretRight == 1) {
        targetTurretAngle -= traverseStep;
      }

      targetTurretAngle = constrain(targetTurretAngle, 0, 180);
    }

    if (fire == 1 && (millis() - lastFired) >= firingCooldown) {
      IrSender.sendNEC(0xF, 32);
      lastFired = millis();
    }
  }
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  if (rightSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(ENA, rightSpeed);
  } else if (rightSpeed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(ENA, -rightSpeed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(ENA, 0);
  }

  if (leftSpeed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    ledcWrite(ENB, leftSpeed);
  } else if (leftSpeed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    ledcWrite(ENB, -leftSpeed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    ledcWrite(ENB, 0);
  }
}
