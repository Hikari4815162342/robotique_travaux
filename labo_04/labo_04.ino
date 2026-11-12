#include <MeAuriga.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_seesaw.h>
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
  NON_CALIB,
  TURN_LINE,
  SEARCH_LINE
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
int turnSpeed = 55;
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

bool firstTimeLab3 = true;
bool firstTimeCal = true;
bool firstTimeLigne = true;
bool firstTimeInvalide = true;
bool firstTimeNotCalibrated = true;

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

void serialEvent(){
  if (!Serial.available()) return;

  receivedData = Serial.readStringUntil('\n');
  parseData(receivedData);
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

  if (!ss.begin()) {
    Serial.println("Erreur de connexion au LyneTracker");
    while (1);
  }
  Serial.println("Connexion réussie au LyneTracker!");
}

void loop() {
  currentTime = millis();
  distance = getDistance(currentTime);
  printAngle(currentTime);
  gyroTask(currentTime);
  sensorsTask(currentTime);
  stateManagerLab4(currentTime);
  encodersTask();
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
      lab3State(ct);
      break;
    case CALIB:
      calibState(ct);
      break;
    case SUIVRE_LIGNE:
      suivreLigneState(ct);
      break;
    case CMD_INVALIDE:
      cmdInvalideState(ct);
      break;
    case NON_CALIB:
      nonCalibState(ct);
      break;
    case TURN_LINE:
      turnLineState(ct);
      break;
    case SEARCH_LINE:
      searchLineState(ct);
      break;
  }
}

void parseData(String& receivedData){
  if (receivedData.length() >= 2){
    if ((uint8_t)receivedData[0] == 0xFF && (uint8_t)receivedData[1] == 0x55){
      receivedData.remove(0, 2);
      receivedData.remove(receivedData.length() - 2, 2);
      if (receivedData == "stop" || curStateLab4 == ATTENTE){
        handleCommand(receivedData);
      }else{
        return;
      }
      
    }
    else if (receivedData.startsWith("!!")){
      receivedData.remove(0, 2);
      receivedData.remove(receivedData.length() - 1, 1);
      if (receivedData == "stop" || curStateLab4 == ATTENTE){
        handleCommand(receivedData);
      }else{
        return;
      }
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
  
  if (cmd != "stop" && cmd != "lab3" && cmd != "ligne" && cmd != "cal"){
    Serial.println("cmd INCONNUE");
  }else{
    Serial.print("cmd reçu : ");
    Serial.println(cmd);
  }

  if (index == -1){
    if (cmd == "stop"){
      stopMotors();
      shutAllLeds();
      firstTimeLab3 = true;
      firstTimeCal = true;
      firstTimeLigne = true;
      firstTimeInvalide = true;
      firstTimeNotCalibrated = true;
      curStateLab4 = ATTENTE;
    }else if (cmd == "lab3"){
      curStateLab4 = LAB3;
    }else if (cmd == "cal"){
      curStateLab4 = CALIB;
    }else if (cmd == "ligne" && !isCalibrated){
      curStateLab4 = NON_CALIB;
    }else if (cmd == "ligne"){
      curStateLab4 = SUIVRE_LIGNE;
    }else{
      curStateLab4 = CMD_INVALIDE;
    } 
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

void ledsBlinkTask(unsigned long ct, int rate, int red, int green, int blue, int index){
  static bool ledState = false;
  static unsigned long lastTime = 0;

   if (ct - lastTime >= rate){
    lastTime = ct;
    for(int i = index; i <= LEDNUM; i++){
      if (ledState){
        led.setColor(i, red, green, blue);
      }else{
        shutAllLeds();
      }
      
    }
    led.show();
    ledState = !ledState;
  }
}

void endState(unsigned long ct) {
  static unsigned long exitTime = 0;
  int exitRate = 3000;
  int rate = 100;

  static bool firstTime = true;

  if (firstTime){
    Serial.println("État: END (LAB3)");
    firstTime = false;
    exitTime = ct + exitRate;
  }

  ledsBlinkTask(ct, rate, 0, 200, 0, 8);

  bool transition = ct > exitTime;
  if (transition){
    firstTime = true;
    done = true;
  }
}


void bootState(unsigned long ct) {
  static unsigned long exitTime = 0;
  int rate = 3000;
  static bool firstTime = true;

  if (firstTime) {
    Serial.println("État: BOOT");
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
    firstTime = true;
  }
}

void lab3State(unsigned long ct){
  if (firstTimeLab3){
    Serial.println("État: LAB3");
    firstTimeLab3 = false;
    curStateLab3 = SEG_A;
  }

  stateManagerLab3(ct);

  bool transitionDone = done;
  if (transitionDone){
    shutAllLeds();
    firstTimeLab3 = true;
    done = false;
    curStateLab4 = ATTENTE;
  }
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
  int speed = 55;

  if (firstTimeCal){
    Serial.println("État: CALIB");
    firstTimeCal = false;
    isCalibrated = false;
    turn(ct, completeTurn, speed, true);
  }

  calibrate();

  bool transitionTurn = turn(ct, completeTurn, speed);
  if (transitionTurn){
    firstTimeCal = true;
    isCalibrated = true;
    curStateLab4 = ATTENTE;
  }
}

int readNormalizedSensor(int index){
  return ((sensors[index].value - sensors[index].minVal) * 1.0) / (sensors[index].maxVal - sensors[index].minVal) * 1000.0;
}

void normalizeValues(){
  for (int i = 0; i < NB_IR; i++){
    sensors[i].normalizedValue = readNormalizedSensor(i);
  }
}

float getLinePosition(){
  float numerator = 0;
  float denominator = 0;

  for (int i = 0; i < NB_IR; i++){
    numerator += sensors[i].normalizedValue * (i - 2);
    denominator += sensors[i].normalizedValue;
  }

  float position = numerator / denominator * 1000;
  return position;
}

float computePID(float position, float consigne = 0.0f) {
    static float kp = 0.35;
    static float ki = 0.01; 
    static float kd = 2;  

    static float integral = 0;
    static float derivative = 0;
    static float lastError = 0;

    float error = position - consigne;

    integral += error;

    const float integralLimit = MAX_PWM;

    integral = constrain(integral, -integralLimit, integralLimit);

    derivative = error - lastError;
    lastError = error;

    float output = kp * error + ki * integral + kd * derivative;

    return output;
}

void followLine(float adjustment, int speed = 40){
  encoderLeft.setTarPWM(speed - adjustment);
  encoderRight.setTarPWM(-speed - adjustment);
}

bool isEndOfLine(){
  for (int i = 0; i < NB_IR; i++){
    int seuil = (sensors[i].minVal + sensors[i].maxVal) / 2;
    if (sensors[i].value > seuil){
      return false;
    }
  }
  return true;
}

bool isLineLost(){
  for (int i = 0; i < NB_IR; i++){
    int seuil = (sensors[i].minVal + sensors[i].maxVal) / 2;
    if (sensors[i].value < seuil){
      return false;
    }
  }
  return true;
}

void suivreLigneState(unsigned long ct){ 
  static unsigned long lineLostTime = 0;
  static bool lineLostTimerStarted = false;
  int rate = 2000;

  if (firstTimeLigne){
    Serial.println("État: SUIVRE_LIGNE");
    firstTimeLigne = false;
  }

  bool transitionLost = isLineLost();

  if (transitionLost){
    if (!lineLostTimerStarted){
      lineLostTime = ct;
      lineLostTimerStarted = true;
    }

    if (ct - lineLostTime >= rate){
      curStateLab4 = SEARCH_LINE;
      lineLostTimerStarted = false;
      firstTimeLigne = true;
      stopMotors();
      return;
    }
  } 
  else {
    lineLostTimerStarted = false;
  }
  
  float consigne = 0.0f;
  normalizeValues();

  float position = getLinePosition();

  float adjustment = computePID(position, consigne);

  followLine(adjustment);

  bool transition = isEndOfLine();
  if (transition){
    stopMotors();
    curStateLab4 = TURN_LINE;
  }
}

void turnLineState(unsigned long ct){
  static bool firstTime = true;
  int speed = 55;
  if (firstTime){
    firstTime = false;
    turn(ct, TURN_180, speed, true);
  }


  bool transition = turn(ct, TURN_180, speed);
  if (transition){
    firstTime = true;
    curStateLab4 = SUIVRE_LIGNE;
  }
}

void searchLineState(unsigned long ct){
  static bool firstTime = true;
  static unsigned long stopBackwardTime = 0;
  if (firstTime){
    firstTime = false;
    stopBackwardTime = ct + 1000;
    turn(ct, 360, 60, true);
  }

  if (ct < stopBackwardTime){
    encoderLeft.setTarPWM(-60);
    encoderRight.setTarPWM(60);
  }else{
    bool turnDone = turn(ct, 360, 60);
    if (turnDone){
      firstTime = true;
      return;
    }

    for (int i = 0; i < NB_IR; i++){
      int seuil = (sensors[i].minVal + sensors[i].maxVal) / 2;
      if (sensors[i].value < seuil){
        firstTime = true;
        curStateLab4 = SUIVRE_LIGNE;
        return;
      }
    }
  }
}

void cmdInvalideState(unsigned long ct){
  static unsigned long exitTime = 0;
  int rate = 100;

  if (firstTimeInvalide){
    firstTimeInvalide = false;
    exitTime = ct + 3000;
  }

  ledsBlinkTask(ct, rate, 200, 0, 0, 0);

  bool transition = ct > exitTime;
  if (transition){
    shutAllLeds();
    firstTimeInvalide = true;
    curStateLab4 = ATTENTE;
  }
}

void nonCalibState(unsigned long ct){
  static unsigned long exitTime = 0;
  int rate = 100;

  if (firstTimeNotCalibrated){
    firstTimeNotCalibrated = false;
    exitTime = ct + 3000;
  }

  ledsBlinkTask(ct, rate, 100, 100, 0, 0);

  bool transition = ct > exitTime;
  if (transition){
    shutAllLeds();
    firstTimeNotCalibrated = true;
    curStateLab4 = ATTENTE;
  }
}

