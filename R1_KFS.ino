#include <Servo.h>

Servo mainServo;      // Main gripper/primary servo
Servo wristServo;     // Wrist servo (0–180 degree control)

int open_angle = 130;     // Main servo open position
int close_angle = 20;     // Main servo close position
int dir = -1;             // Command for main servo (0=open, 1=close)

int wristAngle = -1;      // Target angle for wrist servo

int beltL = 36;           // Belt motor left PWM
int beltR = 37;           // Belt motor right PWM

int armL = 24;            // Arm motor left PWM
int armR = 25;            // Arm motor right PWM

int maxBRPM = 100;        // Max RPM for belt and arm motors
int maxBPWM = 8181;       // Max PWM value (14-bit resolution)

void setup() {
  Serial.begin(9600);
  analogWriteResolution(14);

  mainServo.attach(9);
  wristServo.attach(10);

  mainServo.write(open_angle);
  wristServo.write(90);

  pinMode(beltL, OUTPUT);
  pinMode(beltR, OUTPUT);
  pinMode(armL, OUTPUT);
  pinMode(armR, OUTPUT);

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  stopbelt();
  stoparm();

  Serial.println("Ready");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.peek();

    if (cmd == '0' || cmd == '1') {
      Serial.read();
      dir = cmd - '0';
    }
    else if (cmd == 'b') {
      Serial.read();
      if (Serial.peek() == ' ') Serial.read();
      int rpm = Serial.parseInt();
      controlbelt(rpm);
    }
    else if (cmd == 's') {
      Serial.read();
      if (Serial.peek() == ' ') Serial.read();
      int rpm = Serial.parseInt();
      controlarm(rpm);
    }
    else if (cmd == 'w') {
      Serial.read();
      if (Serial.peek() == ' ') Serial.read();
      wristAngle = Serial.parseInt();
      wristAngle = constrain(wristAngle, 0, 180);
    }
  }

  if (dir == 1) {
    servoclose();
    dir = -1;
  }
  else if (dir == 0) {
    servoopen();
    dir = -1;
  }

  if (wristAngle != -1) {
    wristServo.write(wristAngle);
    wristAngle = -1;
  }
}

void servoopen() {
  for (int i = close_angle; i <= open_angle; i++) {
    mainServo.write(i);
    delay(50);
  }
}

void servoclose() {
  for (int i = open_angle; i >= close_angle; i--) {
    mainServo.write(i);
    delay(50);
  }
}

void controlbelt(int rpm) {
  int pwmVal = map(abs(rpm), 0, maxBRPM, 0, maxBPWM);
  pwmVal = min(pwmVal, maxBPWM);

  if (rpm > 0) {
    analogWrite(beltL, pwmVal);
    analogWrite(beltR, 0);
  }
  else if (rpm < 0) {
    analogWrite(beltL, 0);
    analogWrite(beltR, pwmVal);
  }
  else {
    stopbelt();
  }
}

void controlarm(int rpm) {
  int pwmVal = map(abs(rpm), 0, maxBRPM, 0, maxBPWM);
  pwmVal = min(pwmVal, maxBPWM);

  if (rpm > 0) {
    analogWrite(armL, pwmVal);
    analogWrite(armR, 0);
  }
  else if (rpm < 0) {
    analogWrite(armL, 0);
    analogWrite(armR, pwmVal);
  }
  else {
    stoparm();
  }
}

void stopbelt() {
  analogWrite(beltL, 0);
  analogWrite(beltR, 0);
}

void stoparm() {
  analogWrite(armL, 0);
  analogWrite(armR, 0);
}
