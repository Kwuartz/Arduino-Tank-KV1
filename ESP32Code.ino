#include <BluetoothSerial.h>
#include <ESP32Servo.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>

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
const int turretServoPin = 32;
const int defaultTurretAngle = 90;
int turretAngle = defaultTurretAngle;
Servo TurretServo;

// IR Transmitter
const int transmitterPin = 25;
IRsend Transmitter(transmitterPin);

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

  TurretServo.attach(turretServoPin);
  TurretServo.write(defaultTurretAngle);

  ledcAttach(ENA, freq, resolution);
  ledcAttach(ENB, freq, resolution);
  
  Transmitter.begin();

  Serial.begin(115200);
  SerialBT.begin(bluetoothName);

  Serial.println("Bluetooth Ready");
}

void loop() {
  if (SerialBT.available()) {
    String input = SerialBT.readStringUntil('\n');
    Serial.println("Received: " + input);

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

    if (turretLeft == 1 || turretRight == 1) {
      if (turretLeft == 1) {
        turretAngle -= 1;
      }

      if (turretRight == 1) {
        turretAngle += 1;
      }

      turretAngle = constrain(turretAngle, 0, 180)
      TurretServo.write(turretAngle);
    }

    if (fire == 1) {
      Transmitter.sendNEC(0xF, 32);
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
