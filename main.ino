#include <Servo.h>

// Motor 1
const int ENA = 5;
const int IN1 = 7;
const int IN2 = 8;

// Motor 2
const int ENB = 6;
const int IN3 = 9;
const int IN4 = 10;

// Servos
const int rotationServoPin = 11;
const int firingServoPin = 3;

Servo rotationServo;
Servo firingServo;

// Turret Rotation
const int rotationStep = 10;
int turretRotation = 90;
int minimumThreshold = 32;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  rotationServo.attach(rotationServoPin);
  firingServo.attach(firingServoPin);
  
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    String bluetoothInput = Serial.readStringUntil('\n');

    // CSV format
    int comma1 = bluetoothInput.indexOf(',');
    int comma2 = bluetoothInput.indexOf(',', comma1 + 1);
    int comma3 = bluetoothInput.indexOf(',', comma2 + 1);
    int comma4 = bluetoothInput.indexOf(',', comma3 + 1);

    float distance = bluetoothInput.substring(0, comma1).toFloat();
    float angle = bluetoothInput.substring(comma1 + 1, comma2).toFloat();
    int turretLeft = bluetoothInput.substring(comma2 + 1, comma3).toInt();
    int turretRight = bluetoothInput.substring(comma3 + 1, comma4).toInt();
    int fire = bluetoothInput.substring(comma4 + 1).toInt();
    
    // Speed calculation
    int speed = distance * 255;
    float ratio = cos(angle * 3.14 / 180);

    int leftSpeed, rightSpeed;

    if (angle > 0 and angle < 180) {
      leftSpeed = speed;
      rightSpeed = speed * ratio;
    } else {
      rightSpeed = speed;
      leftSpeed = speed * ratio;
    }
 
    if (leftSpeed < minimumThreshold and leftSpeed > -minimumThreshold) {
      leftSpeed = 0;
    }

    if (rightSpeed < minimumThreshold and rightSpeed > -minimumThreshold) {
      rightSpeed = 0;
    }

    setMotorSpeed(leftSpeed, rightSpeed);

    if (turretLeft == 1) {
      Serial.println(turretRotation);
      turretRotation += rotationStep;
    }

    if (turretRight == 1) {
      Serial.println(turretRotation);
      turretRotation -= rotationStep;
    }

    turretRotation = constrain(turretRotation, 0, 180);
    rotationServo.write(turretRotation);

    if (fire == 1) {
      activateFiringServo();
    }
  }
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  if (rightSpeed > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, rightSpeed);
  } else if (rightSpeed < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -rightSpeed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }

  if (leftSpeed > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, leftSpeed);
  } else if (rightSpeed < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -leftSpeed);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, 0);
  }
}


void activateFiringServo() {
  Serial.println("FIRE");
}