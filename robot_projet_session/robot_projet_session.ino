#include <MeAuriga.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_seesaw.h>

enum State {
  ATTENTE,
  CALIBRATION,
  DEMARRAGE,
  RECHERCHE_DE_LIGNE,
  SUIVI_DE_LIGNE,
  VIRAGE,
  APPROCHE_DU_COLIS,
  DEMI_TOUR,
  ATTENTIF,
  VERIF,
  STATIONNEMENT,
  MANUEL,
  TERMINE
};

State currentState = ATTENTE;

unsigned long currentTime = 0;

const int PULSE = 9;
const double RATIO = 39.267;

const double DIAMETER = 6.5;
const int TICK_TOUR = PULSE * RATIO;
const double CIRC_WHEEL = PI * DIAMETER;

const int TURN_90 = 90;
const int TURN_180 = 180;
int defaultSpeed = 100;
int turnSpeed = 55;
const int MIN_PWM = 35;
const int MAX_PWM = 255;

#define LEDNUM 12
#define LEDPIN 44

MeRGBLed led(PORT0, LEDNUM);
MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);
MeUltrasonicSensor distSensor(PORT_6);
MeGyro gyro(0, 0x69);

double zAngle = 0.0;
double distance = 0;

#define NB_IR 5
bool isCalibrated = false;
Adafruit_seesaw ss;
struct Sensor{
  int minVal = 1023;
  int maxVal = 0;
  int value;
  int normalizedValue;
};
Sensor sensors[NB_IR];

void rightEncoderInterrupt(void) {
  if (digitalRead(encoderRight.getPortB()) == 0) {
    encoderRight.pulsePosMinus();
  } else {
    encoderRight.pulsePosPlus();
  }
}

void leftEncoderInterrupt(void) {
  if (digitalRead(encoderLeft.getPortB()) == 0) {
    encoderLeft.pulsePosMinus();
  } else {
    encoderLeft.pulsePosPlus();
  }
}


void encoderSetup() {
  attachInterrupt(encoderRight.getIntNum(), rightEncoderInterrupt, RISING);
  attachInterrupt(encoderLeft.getIntNum(), leftEncoderInterrupt, RISING);

  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  encoderRight.setPulse(PULSE);
  encoderLeft.setPulse(PULSE);

  encoderRight.setRatio(RATIO);
  encoderLeft.setRatio(RATIO);

  encoderRight.setPosPid(1.8, 0, 1.2);
  encoderLeft.setPosPid(1.8, 0, 1.2);

  encoderRight.setSpeedPid(0.18, 0, 0);
  encoderLeft.setSpeedPid(0.18, 0, 0);
}

void shutAllLeds() {
  led.setColor(0, 0, 0);
  led.show();
}

void setup() {
  Serial.begin(115200);

  led.setpin(LEDPIN);
  encoderSetup();
  gyro.begin();
  shutAllLeds();

  if (!ss.begin()) {
    Serial.println("Erreur de connexion au LyneTracker");
    while (1);
  }
  Serial.println("Connexion réussie au LyneTracker!");
}


void loop() {
  currentTime = millis();
  distance = getDistance(currentTime);
  gyroTask(currentTime);
  sensorsTask(currentTime);
  encodersTask();
}

void encodersTask() {
  encoderRight.loop();
  encoderLeft.loop();
}

double getDistance(unsigned long ct){
  static unsigned long lastTime = 0;
  int rate = 100;
  static double lastDistance = 400;

  if (ct - lastTime < rate){
    return lastDistance;
  }

  lastTime = ct;

  lastDistance = distSensor.distanceCm();

  return lastDistance;
}

void sensorsTask(unsigned long ct){
  static unsigned long lt = 0;
  int rate = 10;

  if (ct - lt < rate) return;

  lt = ct;

  for (int i = 0; i < NB_IR; i++){
    sensors[i].value = ss.analogRead(i);
  }
}

void gyroTask(unsigned long ct){
  static double previousZ = 0.0;
  static bool firstTime = true;

  gyro.update();
  double z = gyro.getAngleZ();

  if (firstTime) {
    firstTime = false;
    previousZ = z;
    return;
  }

  double delta = z - previousZ;

  if (delta > 180) delta -= 360;
  else if (delta < -180) delta += 360;

  zAngle += delta;
  previousZ = z;
}

int limitSpeed(float value, int maxSpeed) {
  value = constrain(value, -maxSpeed, maxSpeed);

  if (fabs(value) < MIN_PWM) {
    value = value < 0 ? -MIN_PWM : MIN_PWM;
  }

  return (int)value;
}

void stopMotors() {
  encoderLeft.setTarPWM(0);
  encoderRight.setTarPWM(0);
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
}

bool turn(unsigned long ct, float goal, int speed = 50, bool init = false) {
  static unsigned long lastTime = 0;
  const unsigned long rate = 20;

  static double zAngleGoal = 0.0;
  static double previousError = 0.0;
  static double errorSum = 0.0;
  float stop = 2.0;

  const double kp = 2.0;
  const double ki = 0.5;
  const double kd = 5.0;

  if (init) {
    lastTime = ct;
    errorSum = 0.0;
    zAngleGoal = zAngle + goal;
    previousError = goal;

    return false;
  }

  if (ct - lastTime < rate) return false;
  
  lastTime = ct;

  double error = zAngleGoal - zAngle;
  if (fabs(error) <= stop) {
    stopMotors();
    return true;
  } 

  errorSum += error;

  double output = kp * error + ki * errorSum + kd * (error - previousError);

  previousError = error;

  int pivotSpeed = limitSpeed(output, speed);
  
  encoderLeft.setTarPWM(pivotSpeed);
  encoderRight.setTarPWM(pivotSpeed);
  
  return false;
}

void calibrate(){
  for (int i = 0; i < NB_IR; i++) {
    if (sensors[i].value < sensors[i].minVal){
      sensors[i].minVal = sensors[i].value;
    }
    if (sensors[i].value > sensors[i].maxVal){
      sensors[i].maxVal = sensors[i].value;
    }
  }
}

void calibState(unsigned long ct){
  int completeTurn = 360;
  static bool firstTime = true;

  if (firstTime){
    Serial.println("État: CALIB");
    turn(ct, completeTurn, turnSpeed, true);
  }

  calibrate();

  bool transitionTurn = turn(ct, completeTurn, turnSpeed);
  if (transitionTurn){
    currentState = ATTENTE;
  }
}


