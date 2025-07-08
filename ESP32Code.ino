#include <BluetoothSerial.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include <IRremote.hpp>

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

// IR Receivers
const int frontReceiverPin = 33;
const int topReceiverPin = 26;
const int backReceiverPin = 27;

IRrecv frontReceiver(frontReceiverPin);
IRrecv topReceiver(topReceiverPin);
IRrecv backReceiver(backReceiverPin);

decode_results results;

// Laser
const int laserPin = 14;

// Firing
const unsigned long firingCooldown = 500; 
unsigned long lastFired = 0;

const uint8_t team = 1;
const uint8_t damage = 1;
const uint32_t firingData = ((uint32_t)team << 8) | damage;

// Hits
const unsigned long hitCooldown = 100;
unsigned long lastHit = 0;

int health = 3;

const int frontDamage = 1;
const int topDamage = 2;
const int backDamage = 3;

// PWM
const int ENA_CHANNEL = 0;
const int ENB_CHANNEL = 1;
const int freq = 1000;
const int resolution = 8;

// Speed
int minimumThreshold = 32;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(laserPin, OUTPUT);
  digitalWrite(laserPin, HIGH);

  ledcSetup(ENA_CHANNEL, freq, resolution);
  ledcSetup(ENB_CHANNEL, freq, resolution);
  ledcAttachPin(ENA, ENA_CHANNEL);
  ledcAttachPin(ENB, ENB_CHANNEL);

  ESP32PWM::allocateTimer(3);
  TurretServo.attach(turretServoPin);

  IrSender.begin(transmitterPin);

  frontReceiver.begin(frontReceiverPin, false, 0);
  topReceiver.begin(topReceiverPin, false, 0);
  backReceiver.begin(backReceiverPin, false, 0);

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

  if ((millis() - lastFired) >= firingCooldown && health >= 0) {
    digitalWrite(laserPin, HIGH);
  } else {
    digitalWrite(laserPin, LOW);
  }

  if (frontReceiver.decode(&results)) {
    uint32_t data = results.value;
    uint8_t receivedTeam = (data >> 8) & 0xFF;

    if ((millis() - lastHit) >= hitCooldown && receivedTeam != team) {
      uint8_t damage = data & 0xFF;
      health -= damage * frontDamage;
      lastHit = millis();
    }

    frontReceiver.resume();
  }

  if (topReceiver.decode(&results)) {
    uint32_t data = results.value;
    uint8_t recievedTeam = (data >> 8) & 0xFF;
    
    if ((millis() - lastHit) >= hitCooldown && recievedTeam != team) {
      uint8_t damage = data & 0xFF;
      health -= damage * topDamage;
      lastHit = millis();
    }

    topReceiver.resume();
  }

  if (backReceiver.decode(&results)) {
    uint32_t data = results.value;
    uint8_t recievedTeam = (data >> 8) & 0xFF;
    
    if ((millis() - lastHit) >= hitCooldown && recievedTeam != team) {
      uint8_t damage = data & 0xFF;
      health -= damage * backDamage;
      lastHit = millis();
    }

    backReceiver.resume();
  }

  if (SerialBT.available() && health > 0) {
    String input = SerialBT.readStringUntil('\n');

    int comma1 = input.indexOf(',');
    int comma2 = input.indexOf(',', comma1 + 1);

    float distance = input.substring(0, comma1).toFloat();
    float angle = input.substring(comma1 + 1, comma2).toFloat();
    int turretLeft = input.substring(comma2 + 1, comma2 + 2).toInt();
    int turretRight = input.substring(comma2 + 2, comma2 + 3).toInt();
    int fire = input.substring(comma2 + 3).toInt();

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

    if (turretLeft == 1 && turretRight == 1) {
      health = 0;
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
      IrSender.sendNEC(firingData, 32);
      lastFired = millis();
    }
  } else {
  }
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  if (rightSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    ledcWrite(ENA_CHANNEL, rightSpeed);
  } else if (rightSpeed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    ledcWrite(ENA_CHANNEL, -rightSpeed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    ledcWrite(ENA_CHANNEL, 0);
  }

  if (leftSpeed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    ledcWrite(ENB_CHANNEL, leftSpeed);
  } else if (leftSpeed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    ledcWrite(ENB_CHANNEL, -leftSpeed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    ledcWrite(ENB_CHANNEL, 0);
  }
}
