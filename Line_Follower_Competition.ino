#include <QTRSensors.h>

// Motor right
#define RIGHT_IN_1 4
#define RIGHT_IN_2 2
#define RIGHT_SPEED 3

#define STBY 6

// Motor left
#define LEFT_IN_1 7
#define LEFT_IN_2 8
#define LEFT_SPEED 9

float x;

// Change values if necessary
float Kp = 0.36; // change if it overshoots (reduce)
float Ki = 0.0;
float Kd = 1.8; // increase if it jerks too much.

int lastError = 0;

int button_calibration = 12;
int button_start = 11;

QTRSensors qtr;

const uint8_t SensorCount = 7;
uint16_t sensorValues[SensorCount];

bool calibrationInProgress = false;
unsigned long calibrationStartTime;

void wheelForward(int speedLeft, int speedRight);
void wheelBackward(int speed);
void calibrateSensors();
void PID_control();

void setup() {
  Serial.begin(9600);

  pinMode(RIGHT_SPEED, OUTPUT);
  pinMode(RIGHT_IN_1, OUTPUT);
  pinMode(RIGHT_IN_2, OUTPUT);
  pinMode(STBY, OUTPUT);

  pinMode(LEFT_IN_1, OUTPUT);
  pinMode(LEFT_IN_2, OUTPUT);
  pinMode(LEFT_SPEED, OUTPUT);

  digitalWrite(STBY, HIGH);

  pinMode(button_calibration, INPUT_PULLUP);
  pinMode(button_start, INPUT_PULLUP);

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A1, A2, A3, A4, A5, A6,A7}, SensorCount);
  qtr.setEmitterPin(13);

  while (!digitalRead(button_calibration)) {
    // Wait for calibration button press
  }

  while (digitalRead(button_calibration)) {
    // Wait for calibration button release
  }

  Serial.println("Calibrating");
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  Serial.println("Calibration Done");

  while (digitalRead(button_start))
    ;

  Serial.println("Running");
}

void loop() {
  PID_control();
}

void calibrateSensors() {
  qtr.calibrate();
}

void PID_control() {
  uint16_t positionLine = qtr.readLineBlack(sensorValues);
  int error = 3500 - positionLine;

  int P = error;
  int I = error + I;  
  int D = error - lastError;
  lastError = error;

  int motorSpeedChange = (P * Kp) + (I * Ki) + (D * Kd);

  Serial.println(motorSpeedChange);

  int motorSpeedA = 250 + motorSpeedChange;
  int motorSpeedB = 250 - motorSpeedChange;

  motorSpeedA = constrain(motorSpeedA, -250, 255);
  motorSpeedB = constrain(motorSpeedB, -250, 255);

  wheelForward(motorSpeedA, motorSpeedB);
}

void wheelForward(int speedLeft, int speedRight) {
  if (speedLeft < 0) {
    speedLeft = -speedLeft;
    digitalWrite(LEFT_IN_1, LOW);
    digitalWrite(LEFT_IN_2, HIGH);
  } else {
    digitalWrite(LEFT_IN_1, HIGH);
    digitalWrite(LEFT_IN_2, LOW);
  }

  if (speedRight < 0) {
    speedRight = -speedRight;
    digitalWrite(RIGHT_IN_1, LOW);
    digitalWrite(RIGHT_IN_2, HIGH);
  } else {
    digitalWrite(RIGHT_IN_1, HIGH);
    digitalWrite(RIGHT_IN_2, LOW);
  }

  analogWrite(RIGHT_SPEED, min(speedRight, 255));  
  analogWrite(LEFT_SPEED, min(speedLeft, 255));   
}


void wheelBackward(int speed) {
  digitalWrite(STBY, HIGH);
  digitalWrite(RIGHT_IN_1, LOW);
  digitalWrite(RIGHT_IN_2, HIGH);
  analogWrite(RIGHT_SPEED, speed);

  digitalWrite(LEFT_IN_1, LOW);
  digitalWrite(LEFT_IN_2, HIGH);
  analogWrite(LEFT_SPEED, speed);
}
