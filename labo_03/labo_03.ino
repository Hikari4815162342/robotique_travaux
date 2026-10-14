#include <MeAuriga.h>
enum State {SETUP, SEG_A, PIVOT_A, SEG_B, PIVOT_B, SEG_C, PIVOT_C, END};
State currentState = SETUP;

const int PULSE = 9;
const double RATIO = 39.267;

const double DIAMETER = 6.5;
const int TOUR = 360;
const double CIRC = PI * DIAMETER;
const short SPEED = 100;

const String DA = "2280684";
unsigned long currentTime = 0;

#define LEDNUM  12
#define LEDPIN  44
const int LED_TOGO = 7;

MeRGBLed led( PORT0, LEDNUM );
MeEncoderOnBoard encoderRight(SLOT1);
MeEncoderOnBoard encoderLeft(SLOT2);
MeGyro gyro(0, 0x69);

void rightEncoderInterrupt(void)
{
  if(digitalRead(encoderRight.getPortB()) == 0)
  {
    encoderRight.pulsePosMinus();
  }
  else
  {
    encoderRight.pulsePosPlus();;
  }
}

void leftEncoderInterrupt(void)
{
  if(digitalRead(encoderLeft.getPortB()) == 0)
  {
    encoderLeft.pulsePosMinus();
  }
  else
  {
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
  
  encoderRight.setPosPid(1.8,0,1.2);
  encoderLeft.setPosPid(1.8,0,1.2);
  
  encoderRight.setSpeedPid(0.18,0,0);
  encoderLeft.setSpeedPid(0.18,0,0);
}

void goStraight(short speed = 100, short firstRun = 0) {
    static double zAngleGoal = 0.0;
    
    static double error = 0.0;
    static double previousError = 0.0;
    static double output = 0;
    
    const double kp = 4.0;
    const double kd = 2.0;
    
    if (firstRun) {
      firstRun = 0;

      zAngleGoal = gyro.getAngleZ();
      
      encoderLeft.runSpeed(speed);
      encoderRight.runSpeed(-speed);
      
      return;
    }
    
    error = gyro.getAngleZ() - zAngleGoal;
    
    output = kp * error + kd * (error - previousError);
    
    previousError = error;        
    
    encoderLeft.runSpeed(speed - output);
    encoderRight.runSpeed(-speed - output);
}

void shutAllLeds(){
  led.setColor(0, 0, 0);
}

void setup() {
  Serial.begin(115200);

  led.setpin(LEDPIN);
  encoderSetup();
  gyro.begin();
  shutAllLeds();
}

void loop() {
  currentTime = millis();
  encodersTask();
  gyro.update();
  stateManager(currentTime);
}

void encodersTask(){
  encoderRight.loop();
  encoderLeft.loop();
}

void stateManager(unsigned long ct){
  switch (currentState){
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
      endState();
      break;
  }
}

void stopMotors(){
  encoderLeft.runSpeed(0);
  encoderRight.runSpeed(0);
}

float getLedProgression(double distance){
  long avgPosition = (abs(encoderLeft.getCurPos()) + abs(encoderRight.getCurPos())) / 2;
  long distanceToGo = distance - avgPosition;
  long ecart = distance - distanceToGo;
  
  return ecart / distance;
}

void setupState(unsigned long ct){
  static unsigned long exitTime = 0;
  int rate = 3000;
  static bool firstTime = true;
  
  if (firstTime){
    Serial.println("ENTRÉE SETUP");
    exitTime = ct + rate;
    firstTime = false;
  }

  bool transition = ct > exitTime;
  if (transition){
    Serial.println("SORTIE SETUP");
    firstTime = true;
    currentState = SEG_A;
  }

}

void segAState(){ 
  static bool firstTime = true;

  if (firstTime){
    Serial.println("ENTRÉE SEGMENT A");
    gyro.update();
    goStraight(100, 1);

    firstTime = false;
  }

  goStraight(); 

  int toGo = 4 * 59;
  double distance = toGo/CIRC * TOUR;
  float progression = getLedProgression(distance);
  int nbLeds = round(progression * LED_TOGO);
  

  for (int i = 1; i <= nbLeds; i++){
    led.setColor(i, 50, 50, 50);
  }
  led.show();

  int offset = 0.02 * distance;
  long avgPosition = (abs(encoderLeft.getCurPos()) + abs(encoderRight.getCurPos())) / 2;
  bool transition = avgPosition >= distance - offset;

  if (transition){
    Serial.println("SORTIE SEGMENT A");
    stopMotors();
    led.setColor(LEDNUM, 0, 200, 0);
    led.show();
    firstTime = true;
    currentState = PIVOT_A;
  }
}

void pivotAState(){

}

void segBState(){

}

void pivotBState(){

}

void segCState(){

}

void pivotCState(){

}

void endState(){

}