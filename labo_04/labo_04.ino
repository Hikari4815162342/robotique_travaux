#include <MeAuriga.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
enum StateLab3 { 
  SEG_A,
  PIVOT_A,
  SEG_B,
  PIVOT_B,
  SEG_C,
  PIVOT_C,
  END 
};
StateLab3 curStateLab3 = SEG_A;

enum StateLab4 {
  BOOT,
  ATTENTE,
  LAB3,
  CALIB,
  SUIVRE_LIGNE,
  CMD_INVALIDE,
  NON_CALIB
};
StateLab4 curStateLab4 = BOOT;

String receivedData = "";

const int PULSE = 9;
const double RATIO = 39.267;

const double DIAMETER = 6.5;
const int TICK_TOUR = PULSE * RATIO;
const double CIRC_WHEEL = PI * DIAMETER;

const int TURN_90 = 90;
const int TURN_180 = 180;
int defaultSpeed = 100;
int turnSpeed = 60;
int toGo = 120;
const int MIN_PWM = 35;
const int MAX_PWM = 255;

const int DA = 2280684;
unsigned long currentTime = 0;
bool done = false;

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

double zAngle = 0.0;

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

void shutAllLeds() {
  led.setColor(0, 0, 0);
  led.show();
}

void serialEvent(){
  if (!Serial.available()) return;

  receivedData = Serial.readStringUntil('\n');
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
  currentTime = millis();
  distance = getDistance(currentTime);
  printAngle(currentTime);
  gyroTask(currentTime);
  //stateManagerLab3(currentTime);
  encodersTask();
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
  display.println(zAngle);
  display.display();
}

void encodersTask() {
  encoderRight.loop();
  encoderLeft.loop();
}

void stateManagerLab3(unsigned long ct) {
  switch (curStateLab3) {
    case SEG_A:
      segAState();
      break;
    case PIVOT_A:
      pivotAState(ct);
      break;
    case SEG_B:
      segBState();
      break;
    case PIVOT_B:
      pivotBState(ct);
      break;
    case SEG_C:
      segCState();
      break;
    case PIVOT_C:
      pivotCState(ct);
      break;
    case END:
      endState(ct);
      break;
  }
}

void stateManagerLab4(unsigned long ct){
  switch (curStateLab4){
    case BOOT:
      bootState(ct);
      break;
    case ATTENTE:
      attenteState();
      break;
    case LAB3:
      lab3State();
      break;
    case CALIB:
      calibState();
      break;
    case SUIVRE_LIGNE:
      suivreLigneState();
      break;
    case CMD_INVALIDE:
      cmdInvalideState();
      break;
    case NON_CALIB:
      nonCalibState();
      break;
  }
}


void parseData(String& receivedData){
  if (receivedData.length() >= 2){
    if ((uint8_t)receivedData[0] == 0xFF && (uint8_t)receivedData[1] == 0x55){
      receivedData.remove(0, 2);
      handleCommand(receivedData);
    }
    else if (receivedData.startsWith("!!")){
      receivedData.remove(0, 2);
      handleCommand(receivedData);
    }
    else{
      Serial.print(F("Données non reconnues : "));
      Serial.println(receivedData);
      return;
    }
  }
  else{
    Serial.print(F("Données trop courtes : "));
    Serial.println(receivedData);
    return;
  }
}


void handleCommand(const String& cmd){
  int index = cmd.indexOf(',');
  if (index == -1){
    // Commande sans paramètre
    if (cmd == "FOLLOW_LINE"){
      currentState = FOLLOW_LINE;
    }
    else if (cmd == "STOP"){
      currentState = STOP;
    }
    else {
      currentState = ETC;
    }
  }
}

void bootState(unsigned long ct) {
  static unsigned long exitTime = 0;
  int rate = 3000;
  static bool firstTime = true;

  if (firstTime) {
    Serial.println("État: ATTENTE");
    exitTime = ct + rate;
    firstTime = false;
  }

  bool transition = ct > exitTime;
  if (transition) {
    firstTime = true;
    curStateLab4 = ATTENTE;
  }
}

void attenteState(){
  static bool firstTime = true;
  if (firstTime){
    Serial.println("État: ATTENTE");
    firstTime = false;
    done = false;
  }

  static String data = "";
  bool transition = data != receivedData;
  if (transition){
    data = receivedData;
    parseData(data);
  }

}

void lab3State(){

}

void calibState(){

}

void suivreLigneState(){

}

void cmdInvalideState(){

}

void nonCalibState(){

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
  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
}

float getLedProgressionA(double distance) {
  long avgPosition = (abs(encoderLeft.getPulsePos()) + abs(encoderRight.getPulsePos())) / 2;
  long distanceToGo = distance - avgPosition;
  long gap = distance - distanceToGo;

  return gap / distance;
}

void goStraight(short speed = 100, short firstRun = 0) {
  static double zAngleGoal = 0.0;

  static double previousError = 0.0;

  const double kp = 4.0;
  const double kd = 1.0;

  if (firstRun) {
    firstRun = 0;

    zAngleGoal = zAngle;

    encoderLeft.setTarPWM(speed);
    encoderRight.setTarPWM(-speed);

    return;
  }

  double error = zAngle - zAngleGoal;

  double output = kp * error + kd * (error - previousError);

  previousError = error;

  encoderLeft.setTarPWM(speed - output);
  encoderRight.setTarPWM(-speed - output);
}

void segAState() {
  static bool firstTime = true;

  if (firstTime) {
    Serial.println("État: SEG_A (LAB3)");
    resetDistance();
    goStraight(defaultSpeed, 1);

    firstTime = false;
  }

  goStraight();

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
    stopMotors();
    led.setColor(ledsDone--, 0, 200, 0);
    led.show();
    
    firstTime = true;
    curStateLab3 = PIVOT_A;
  }
}

int turnDirection() {
  static int dir = 1;
  switch (curStateLab3){
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
    case PIVOT_C:
      dir = 1;
      break;
  }
  return dir;
}

int limitSpeed(float value, int maxSpeed) {
  value = constrain(value, -maxSpeed, maxSpeed);

  if (fabs(value) < MIN_PWM) {
    value = value < 0 ? -MIN_PWM : MIN_PWM;
  }

  return (int)value;
}

bool turn(unsigned long ct, float goal, int speed = 100, bool init = false) {
  static unsigned long lastTime = 0;
  const unsigned long rate = 20;

  static double zAngleGoal = 0.0;
  static double previousError = 0.0;
  static double errorSum = 0.0;
  float stop = 1.0;

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

void pivotAState(unsigned long ct) {
  static bool firstTime = true;
  static float targetAngle = 0.0;
  static int targetSpeed = 0;

  if (firstTime) {
    Serial.println("État: PIVOT_A (LAB3)");
    firstTime = false;
    shutProgressionLeds();
    resetDistance(); 
    targetAngle = TURN_90 * turnDirection();
    targetSpeed = constrain(turnSpeed, MIN_PWM, MAX_PWM);

    turn(ct, targetAngle, targetSpeed, true);
  }

  bool transition = turn(ct, targetAngle, targetSpeed);

  if (transition) {

    firstTime = true;
    curStateLab3 = SEG_B;
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
  double stopDist = 32.00;
  int slowSpeed = defaultSpeed / 2;
  static double startDistance = 0.0;

  if (firstTime) {
    Serial.println("État: SEG_B (LAB3)");
    resetDistance();
    shutProgressionLeds();
    startDistance = distance;
    goStraight(defaultSpeed, 1);

    firstTime = false;
  }

  if (distance > slowDownDist){
    goStraight(defaultSpeed, 0);
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
    stopMotors();
    led.setColor(ledsDone--, 0, 200, 0);
    led.show();

    firstTime = true;
    curStateLab3 = PIVOT_B;
  }
}

void pivotBState(unsigned long ct) {
  static bool firstTime = true;
  static float targetAngle = 0.0;
  static int targetSpeed = 0;

  if (firstTime) {
    Serial.println("État: PIVOT_B (LAB3)");
    firstTime = false;
    shutProgressionLeds();
    resetDistance(); 
    targetAngle = TURN_90 * turnDirection();
    targetSpeed = constrain(turnSpeed, MIN_PWM, MAX_PWM);

    turn(ct, targetAngle, targetSpeed, true);
  }

  bool transition = turn(ct, targetAngle, targetSpeed);

  if (transition) {

    firstTime = true;
    curStateLab3 = SEG_C;
  }
}

void segCState() {
  static bool firstTime = true;

  if (firstTime) {
    Serial.println("État: SEG_C (LAB3)");
    shutProgressionLeds();
    resetDistance();
    goStraight(100, 1);

    firstTime = false;
  }

  goStraight();

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
    stopMotors();
    led.setColor(ledsDone--, 0, 200, 0);
    led.show();
    firstTime = true;
    curStateLab3 = PIVOT_C;
  }
}

void pivotCState(unsigned long ct) {
  static bool firstTime = true;
  static float targetAngle = 0.0;
  static int targetSpeed = 0;

  if (firstTime) {
    Serial.println("État: PIVOT_C (LAB3)");
    firstTime = false;
    shutProgressionLeds();
    resetDistance(); 
    targetAngle = TURN_180 * turnDirection();
    targetSpeed = constrain(turnSpeed, MIN_PWM, MAX_PWM);

    turn(ct, targetAngle, targetSpeed, true);
  }

  bool transition = turn(ct, targetAngle, targetSpeed);

  if (transition) {

    firstTime = true;
    curStateLab3 = END;
  }
}

void endState(unsigned long ct) {
  static unsigned long exitTime = 0;
  int exitRate = 3000;
  static unsigned long lastTime = 0;
  int rate = 100;
  static bool ledState = false;

  static bool firstTime = true;

  if (firstTime){
    Serial.println("État: END (LAB3)");
    firstTime = false;
    exitTime = ct + exitRate;
  }

  if (ct - lastTime >= rate){
    lastTime = ct;
    for(int i = 8; i <= LEDNUM; i++){
      if (ledState){
        led.setColor(i, 0, 200, 0);
      }else{
        shutAllLeds();
      }
      
    }
    led.show();
    ledState = !ledState;
  }

  bool transition = ct > exitTime;
  if (transition){
    firstTime = true;
    done = true;
  }
}
