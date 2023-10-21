#include <QTRSensors.h>

#define RIGHT_SPEED 3
#define RIGHT_IN_1 4
#define RIGHT_IN_2 2
#define STBY 6
#define LEFT_IN_1 7
#define LEFT_IN_2 8
#define LEFT_SPEED 9

float Kp = 0.37;
float Ki = 0.0002;
float Kd = 6.71;

QTRSensors qtr;

int button_stop = 10;
int button_start = 11;
int button_calibration = 12;

bool buttonState = 0;
bool lastButtonState = 0;

const uint8_t SensorPins[] = {A1, A2, A3, A4, A5, A6};
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

int lastError = 0;

void wheelControl(int speedLeft, int speedRight);
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
  qtr.setSensorPins(SensorPins, SensorCount);
  qtr.setEmitterPin(13);

  while (!digitalRead(button_calibration)) {
    // Wait for the calibration button to be released
  }


  while (digitalRead(button_calibration)) {
    // Wait for the calibration button to be released
  }

  Serial.println("Calibrating");
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  Serial.println("Calibration Done");

  while (digitalRead(button_start)) {
    // Wait until the start button is pressed
  }

  Serial.println("Running");
}

void loop() 
{
  PID_control();
}

void PID_control() {
  uint16_t positionLine = qtr.readLineBlack(sensorValues);
  int error = 3500 - positionLine;

  int P = error;
  static int I = 0; // Initialize as a static variable
  int D = error - lastError;
  lastError = error;

  int motorSpeedChange = (P * Kp) + (I * Ki) + (D * Kd);

  int motorSpeedA = 255 + motorSpeedChange;
  int motorSpeedB = 255 - motorSpeedChange;

  // Ensure motorSpeedA and motorSpeedB are within valid range
  motorSpeedA = constrain(motorSpeedA, -150, 255);
  motorSpeedB = constrain(motorSpeedB, -150, 255);

  wheelControl(motorSpeedA, motorSpeedB);
}

void wheelControl(int speedLeft, int speedRight) {
  // Simplify the wheel control logic
  digitalWrite(LEFT_IN_1, speedLeft >= 0 ? HIGH : LOW);
  digitalWrite(LEFT_IN_2, speedLeft >= 0 ? LOW : HIGH);
  analogWrite(LEFT_SPEED, abs(speedLeft));

  digitalWrite(RIGHT_IN_1, speedRight >= 0 ? HIGH : LOW);
  digitalWrite(RIGHT_IN_2, speedRight >= 0 ? LOW : HIGH);
  analogWrite(RIGHT_SPEED, abs(speedRight));
}

void wheelStop()
{
  Serial.println("Stop");
  digitalWrite(LEFT_IN_1, LOW);
  digitalWrite(LEFT_IN_2, LOW);

  digitalWrite(RIGHT_IN_1, LOW);
  digitalWrite(RIGHT_IN_2, LOW);
  
}

void calibrateSensors() {
  qtr.calibrate();
}
