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
  VERIFICATION,
  STATIONNEMENT,
  MANUEL,
  TERMINE
};

State currentState = ATTENTE;
State requestedState = ATTENTE;

unsigned long currentTime = 0;

const int PULSE = 9;
const double RATIO = 39.267;

const double DIAMETER = 6.5;
const int TICK_TOUR = PULSE * RATIO;
const double CIRC_WHEEL = PI * DIAMETER;

const int TURN_90 = 90;
const int TURN_180 = 180;
int speed = 50;
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

const int MAX_PARAMS = 10;

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

void serialEvent() { 
  static String receivedData = "";

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
  stateManager(currentTime);
  encodersTask();
}

void parseData(String& receivedData){
  if (receivedData.length() >= 2){
    if ((uint8_t)receivedData[0] == 0xFF && (uint8_t)receivedData[1] == 0x55){
      receivedData.remove(0, 2);
      receivedData.remove(receivedData.length() - 2, 2);
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

void setSpeed(int value){
  Serial.print("j'ai exécuté p,");
  Serial.println(value);

  speed = value;
}

void forward(){
  Serial.println("j'ai exécuté W");
}

void left(){
  Serial.println("j'ai exécuté A");
}

void backward(){
  Serial.println("j'ai exécuté S");
}

void right(){
  Serial.println("j'ai exécuté D");
}

void handleCommand(const String& cmd) {

  int index = cmd.indexOf(',');
  if (index == -1) {
    if (cmd == "E"){
      klaxon();
    }
    else if (cmd == "C"){
      Serial.print("j'ai exécuté C");
      requestedState = CALIBRATION;
    }
    else if (cmd == "GO"){
      Serial.print("j'ai exécuté GO");
      requestedState = SUIVI_DE_LIGNE;
    }
    else if (cmd == "STOP"){
      Serial.print("j'ai exécuté STOP");
      requestedState = ATTENTE;
    }
    else if (cmd == "MAN"){
      Serial.print("j'ai exécuté MAN");
      requestedState = MANUEL;
    }
    else if (currentState == MANUEL){
      if (cmd == "W"){
        forward();
      }
      else if (cmd == "A"){
        left();
      }
      else if (cmd == "S"){
        backward();
      }
      else if (cmd == "D"){
        right();
      }
    }
  }
  else {
    String leftPart = cmd.substring(0, index);

    if (leftPart == "p"){
      String rightPart = cmd.substring(index + 1);
      int value = rightPart.toInt();
      setSpeed(value);
    }
    else if (leftPart == "l"){
      String params[MAX_PARAMS];
      int begin = 2;
      int comma = 0;
      
      int countParams = 0;
    
      while (comma != -1) {
          comma = cmd.indexOf(',', begin);
          params[countParams] = cmd.substring(begin, comma);
          begin = comma;
          begin++;
          countParams++;
      }

      if (countParams == 3){
        int r = params[0].toInt();
        int g = params[1].toInt();
        int b = params[2].toInt();
        led.setColor(r, g, b);
      }else if (countParams == 4){
        int i = params[0].toInt();
        int r = params[1].toInt();
        int g = params[2].toInt();
        int b = params[3].toInt();
        led.setColorAt(i, r, g, b);
      }
      led.show();

      Serial.print("j'ai exécuté ");
      Serial.println(cmd);
    }
  }
}

void klaxon(){
  Serial.println("j'ai exécuté E");
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

bool turn(unsigned long ct, float goal, bool init = false) {
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


void stateManager(unsigned long ct){
  switch (currentState){
    case ATTENTE:
      attenteState();
      break;
    case CALIBRATION:
      calibrationState(ct);
      break;
    case DEMARRAGE:
      demarrageState();
      break;
    case RECHERCHE_DE_LIGNE:
      rechercheDeLigneState();
      break;
    case SUIVI_DE_LIGNE:
      suiviDeLigneState();
      break;
    case VIRAGE:
      virageState();
      break;
    case APPROCHE_DU_COLIS:
      approcheDuColisState();
      break;
    case DEMI_TOUR:
      demiTourState(ct);
      break;
    case ATTENTIF:
      attentifState();
      break;
    case VERIFICATION:
      verificationState();
      break;
    case STATIONNEMENT:
      stationnementState();
      break;
    case MANUEL:
      manuelState();
      break;
    case TERMINE:
      termineState();
      break;
  }
}

void attenteState(){
  static bool firstTime = true;
  if (firstTime){
    Serial.print("État: ");
    Serial.println(currentState);
    firstTime = false;
  }

  bool transition = requestedState != currentState;
  if (transition){
    firstTime = true;
    currentState = requestedState;
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

void calibrationState(unsigned long ct){
  int completeTurn = 360;
  static bool firstTime = true;

  if (firstTime){
    Serial.print("État: ");
    Serial.println(currentState);
    turn(ct, completeTurn, true);
  }

  calibrate();

  bool transitionTurn = turn(ct, completeTurn);
  if (transitionTurn){
    currentState = ATTENTE;
  }
}

void demarrageState(){

}

void rechercheDeLigneState(){

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

void setAdjustment(float adjustment){
  encoderLeft.setTarPWM(speed - adjustment);
  encoderRight.setTarPWM(-speed - adjustment);
}

void followLine(){
  float consigne = 0.0f;
  normalizeValues();

  float position = getLinePosition();

  float adjustment = computePID(position, consigne);

  setAdjustment(adjustment);
}

void suiviDeLigneState(){ 
  static bool firstTime = true;
  int distanceThreshold = 30;

  if (firstTime){
    Serial.print("État: ");
    Serial.println(currentState);
    firstTime = false;
  }

  followLine();

  bool transitionObjet = distance <= distanceThreshold;
  if (transitionObjet){
    currentState = ATTENTIF;
  }

  bool transitionStop = requestedState == ATTENTE;
  if (transitionStop){
    stopMotors();
    currentState = requestedState;
  }
}

void virageState(){}
void approcheDuColisState(){}

void demiTourState(unsigned long ct){
  static bool firstTime = true;
  if (firstTime){
    Serial.print("État: ");
    Serial.println(currentState);
    firstTime = false;
    turn(ct, TURN_180, true);
  }


  bool transition = turn(ct, TURN_180);
  if (transition){
    firstTime = true;
    currentState = SUIVI_DE_LIGNE;
  }
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


void attentifState(){
  static bool firstTime = true;
  if (firstTime){
    Serial.print("État: ");
    Serial.println(currentState);
    firstTime = false;
  }

  followLine();

  bool transition = isEndOfLine();
  if (transition){
    stopMotors();
    currentState = DEMI_TOUR;
  }
  
}
void verificationState(){}
void stationnementState(){}
void manuelState(){
  
}
void termineState(){}



