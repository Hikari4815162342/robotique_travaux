#include <MeAuriga.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
enum State { SETUP,
             SEG_A,
             PIVOT_A,
             SEG_B,
             PIVOT_B,
             SEG_C,
             PIVOT_C,
             END };
State currentState = SETUP;

const int PULSE = 9;
const double RATIO = 39.267;

const double DIAMETER = 6.5;
const int TICK_TOUR = PULSE * RATIO;
const double CIRC_WHEEL = PI * DIAMETER;
const double CIRC_ROBOT = 48.7;

const double QUARTER = CIRC_ROBOT / 4.0;
const double NB_TOURS = QUARTER / CIRC_WHEEL;
const double TICKS_FOR_90 = NB_TOURS * TICK_TOUR;

const int TURN_90 = 90;
const int TURN_180 = 180;
int speed = 100;

const int DA = 2280684;
unsigned long currentTime = 0;

#define LEDNUM 12
#define LEDPIN 44
const int LED_TOGO = 7;
const int FIRST_LED = 1;

#define SCREEN_HEIGHT 64
#define SCREEN_WIDTH 128
#define OLED_RESET -1

#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

MeRGBLed led(PORT0, LEDNUM);
MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);
MeUltrasonicSensor distSensor(PORT_6);
MeGyro gyro(0, 0x69);

double distance = 0;
int ledsDone = LEDNUM;

void rightEncoderInterrupt(void) {
  if (digitalRead(encoderRight.getPortB()) == 0) {
    encoderRight.pulsePosMinus();
  } else {
    encoderRight.pulsePosPlus();
    ;
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


void goStraight(short speed = 100, short firstRun = 0) {
  static double zAngleGoal = 0.0;

  static double error = 0.0;
  static double previousError = 0.0;
  static double output = 0;

  const double kp = 4.0;
  const double kd = 1.0;

  if (firstRun) {
    firstRun = 0;

    zAngleGoal = gyro.getAngleZ();

    encoderLeft.setTarPWM(speed);
    encoderRight.setTarPWM(-speed);

    return;
  }

  error = gyro.getAngleZ() - zAngleGoal;

  output = kp * error + kd * (error - previousError);

  previousError = error;

  encoderLeft.setTarPWM(speed - output);
  encoderRight.setTarPWM(-speed - output);
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

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Échec d'allocation SSD1306"));
    for (;;)
      ;
  }

  display.clearDisplay();
  display.display();
}

void loop() {
  //Serial.println(gyro.getAngleZ());
  currentTime = millis();
  distance = getDistance(currentTime);
  printAngle(currentTime);
  gyro.update();
  stateManager(currentTime);
  encodersTask();
}

void printAngle(unsigned long ct) {
  static unsigned long lt = 0;
  int rate = 1000;

  if (ct - lt < rate) {
    return;
  }

  lt = ct;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Angle: ");
  display.print(gyro.getAngleZ());
  display.print(" ---  Distance: ");
  display.print(distance);
  display.println(" cm");
  display.display();
}

void encodersTask() {
  encoderRight.loop();
  encoderLeft.loop();
}

void stateManager(unsigned long ct) {
  switch (currentState) {
    case SETUP:
      setupState(ct);
      break;
    case SEG_A:
      segAState();
      break;
    case PIVOT_A:
      pivotAState();
      break;
    case SEG_B:
      segBState();
      break;
    case PIVOT_B:
      pivotBState();
      break;
    case SEG_C:
      segCState();
      break;
    case PIVOT_C:
      pivotCState();
      break;
    case END:
      endState(ct);
      break;
  }
}

void shutProgressionLeds() {
  for (int i = 1; i <= LED_TOGO; i++) {
    led.setColor(i, 0, 0, 0);
  }
  led.show();
}

void resetDistance() {
  encoderLeft.setPulsePos(0);
  encoderRight.setPulsePos(0);
}

void stopMotors() {
  encoderLeft.setTarPWM(0);
  encoderRight.setTarPWM(0);
}

float getLedProgressionA(double distance) {
  long avgPosition = (abs(encoderLeft.getPulsePos()) + abs(encoderRight.getPulsePos())) / 2;
  long distanceToGo = distance - avgPosition;
  long gap = distance - distanceToGo;

  return gap / distance;
}

void setupState(unsigned long ct) {
  static unsigned long exitTime = 0;
  int rate = 3000;
  static bool firstTime = true;

  if (firstTime) {
    Serial.println("ENTRÉE SETUP");
    exitTime = ct + rate;
    firstTime = false;
  }

  bool transition = ct > exitTime;
  if (transition) {
    Serial.println("SORTIE SETUP");

    firstTime = true;
    currentState = SEG_A;
  }
}

void segAState() {
  static bool firstTime = true;

  if (firstTime) {
    Serial.println("ENTRÉE SEGMENT A");
    resetDistance();
    goStraight(100, 1);

    firstTime = false;
  }

  goStraight();

  int toGo = 2 * 59;
  double distance = toGo / CIRC_WHEEL * TICK_TOUR;
  float progression = getLedProgressionA(distance);
  int nbLeds = round(progression * LED_TOGO);

  for (int i = 1; i <= nbLeds; i++) {
    led.setColor(i, 50, 50, 50);
  }
  led.show();

  long avgPosition = (abs(encoderLeft.getPulsePos()) + abs(encoderRight.getPulsePos())) / 2;

  bool transition = avgPosition >= distance;
  if (transition) {
    Serial.println("SORTIE SEGMENT A");
    stopMotors();
    led.setColor(ledsDone--, 0, 200, 0);
    led.show();
    
    firstTime = true;
    currentState = PIVOT_A;
  }
}

int turnDirection() {
  static int dir = 1;
  switch (currentState){
    case PIVOT_A:
      if (DA % 2 == 0) {
        dir = -1;
      } else {
        dir = 1;
      }
      break;
    case PIVOT_B: 
      if (DA % 2 == 0) {
        dir = 1;
      } else {
        dir = -1;
      }
      break;
  }
  return dir;
}

bool turn(double goal) {
  static double error = 0.0;
  static double previousError = 0.0;
  static double output = 0;
  int stop = 1;
  int minSpeed = speed / 2;

  const double kp = 2.75;
  const double kd = 1.0;

  error = goal - gyro.getAngleZ();

  output = kp * error + kd * (error - previousError);

  previousError = error;

  if (abs(error) <= stop) {
    stopMotors();
    return true;
  } else {
    if (output < minSpeed){
      output = minSpeed * turnDirection();
    }

    encoderLeft.setTarPWM(output);
    encoderRight.setTarPWM(output);
    return false;
  }
}

void pivotAState() {
  static bool firstTime = true;
  static double targetAngle = 0.0;
  if (firstTime) {
    firstTime = false;
    shutProgressionLeds();
    resetDistance(); 
    targetAngle = gyro.getAngleZ() + (TURN_90 * turnDirection());
    encoderLeft.moveTo(TICKS_FOR_90 * turnDirection());
    encoderRight.moveTo(TICKS_FOR_90 * turnDirection());
  }

  if (!encoderLeft.isTarPosReached() || !encoderRight.isTarPosReached()) {
    return;
  }
  
  bool transition = turn(targetAngle);

  if (transition) {
    Serial.println("SORTIE PIVOT A");

    firstTime = true;
    currentState = SEG_B;
  }
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


void segBState() {
  static bool firstTime = true;
  double slowDownDist = 45.0;
  double stopDist = 35.0;
  int slowSpeed = speed / 2;
  static double startDistance = 0.0;

  if (firstTime) {
    Serial.println("ENTRÉE SEGMENT B");
    resetDistance();
    shutProgressionLeds();
    startDistance = distance;
    goStraight(speed, 1);

    firstTime = false;
  }

  if (distance > slowDownDist){
    goStraight(speed, 0);
  }else{
    goStraight(slowSpeed, 0);
  }

  double nbLeds = map(distance, startDistance, stopDist, FIRST_LED, LED_TOGO);
  for (int i = 1; i <= nbLeds; i++) {
    led.setColor(i, 50, 50, 50);
  }
  led.show();

  bool transition = distance <= stopDist;
  if (transition){
    Serial.println("SORTIE SEGMENT B");
    stopMotors();
    led.setColor(ledsDone--, 0, 200, 0);
    led.show();

    firstTime = true;
    currentState = PIVOT_B;
  }
}

void pivotBState() {
  static bool firstTime = true;
  static double targetAngle = 0.0;
  if (firstTime) {
    firstTime = false;
    shutProgressionLeds();
    resetDistance(); 
    targetAngle = gyro.getAngleZ() + (TURN_90 * turnDirection());
    encoderLeft.moveTo(TICKS_FOR_90 * turnDirection());
    encoderRight.moveTo(TICKS_FOR_90 * turnDirection());
  }

  if (!encoderLeft.isTarPosReached() || !encoderRight.isTarPosReached()) {
    return;
  }
  
  bool transition = turn(targetAngle);

  if (transition) {
    Serial.println("SORTIE PIVOT A");

    firstTime = true;
    currentState = SEG_C;
  }
}

void segCState() {
  static bool firstTime = true;

  if (firstTime) {
    Serial.println("ENTRÉE SEGMENT C");
    shutProgressionLeds();
    resetDistance();
    goStraight(100, 1);

    firstTime = false;
  }

  goStraight();

  int toGo = 2 * 59;
  double distance = toGo / CIRC_WHEEL * TICK_TOUR;
  float progression = getLedProgressionA(distance);
  int nbLeds = round(progression * LED_TOGO);

  for (int i = 1; i <= nbLeds; i++) {
    led.setColor(i, 50, 50, 50);
  }
  led.show();

  long avgPosition = (abs(encoderLeft.getPulsePos()) + abs(encoderRight.getPulsePos())) / 2;

  bool transition = avgPosition >= distance;
  if (transition) {
    Serial.println("SORTIE SEGMENT C");
    stopMotors();
    led.setColor(ledsDone--, 0, 200, 0);
    led.show();
    firstTime = true;
    currentState = PIVOT_C;
  }
}

void pivotCState() {
  static bool firstTime = true;
  static double targetAngle = 0.0;
  if (firstTime) {
    firstTime = false;
    shutProgressionLeds();
    resetDistance(); 
    targetAngle = gyro.getAngleZ() + (TURN_180 * turnDirection());
    encoderLeft.moveTo((2 * TICKS_FOR_90) * turnDirection());
    encoderRight.moveTo((2 * TICKS_FOR_90) * turnDirection());
  }

  if (!encoderLeft.isTarPosReached() || !encoderRight.isTarPosReached()) {
    return;
  }
  
  bool transition = turn(targetAngle);

  if (transition) {
    firstTime = true;
    currentState = END;
  }
}

void endState(unsigned long ct) {
  static unsigned long lastTime = 0;
  int rate = 100;

  if (ct - lastTime >= rate){
    lastTime = ct;
    for(int i = 8; i <= LEDNUM; i++){
      led.setColor(i, 0, 200, 0);
    }
    led.show();
  }else{
    shutAllLeds();
  }

}