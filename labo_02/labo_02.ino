#include <MeAuriga.h>

#define LEDNUM  12
#define LEDPIN  44

enum RobotState {DRIVE, SLOW, STOP, BACKWARD, TURN};
RobotState currentState;

MeRGBLed led(PORT0, LEDNUM);
MeUltrasonicSensor distSensor(PORT_10);

const int MAX_SPEED = 255;

const int ML_PWM = 11;
const int ML_IN1 = 48;
const int ML_IN2 = 49; 

const int MR_PWM = 10;
const int MR_IN1 = 47;
const int MR_IN2 = 46; 

int red = 0;
int green = 0;
int blue = 0;
int speed = 200;

const int DISTANCE_SWITCH = 100;
const int DISTANCE_LIMIT = 30;
const int WAIT_TIME = 2000;
unsigned long currentTime = 0;
double distance = 0;

void setup() {
  Serial.begin(115200);
  
  led.setpin(LEDPIN);

  pinMode(ML_PWM, OUTPUT); 
  pinMode(ML_IN1, OUTPUT); 
  pinMode(ML_IN2, OUTPUT);

  pinMode(MR_PWM, OUTPUT);  
  pinMode(MR_IN1, OUTPUT);  
  pinMode(MR_IN2, OUTPUT);

  currentState = DRIVE;
}

void loop() {
  currentTime = millis();
  distance = getDistance(currentTime);
  distancePrintTask(currentTime, distance);
  stateManager(currentTime, distance);
}

void resetLeds(){
  red = 0;
  green = 0;
  blue = 0;
  led.setColor(red, green, blue);
  led.show();
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

void distancePrintTask(unsigned long ct, double dist){
  static unsigned long lastTime = 0;
  int rate = 250;

  if (ct - lastTime < rate) {
    return;
  }

  lastTime = ct;
    
  Serial.print("Distance: ");
  Serial.print(dist);
  Serial.println(" cm");
  Serial.print("Vitesse : ");
  Serial.println(speed);
}

void stateManager(unsigned long ct, double dist){
  switch(currentState){
    case DRIVE:
      drive(dist);
      break;
    case SLOW:
      slow(dist);
      break;
    case STOP:
      stop(ct);
      break;
    case BACKWARD:
      backward(ct);
      break;
  }
}

void drive(double dist){
  static bool firstTime = true;
  if (firstTime){
    speed = 200;
    digitalWrite(ML_IN2, LOW);
    digitalWrite(ML_IN1, HIGH);
    analogWrite(ML_PWM, speed);

    digitalWrite(MR_IN2, LOW);
    digitalWrite(MR_IN1, HIGH);
    analogWrite(MR_PWM, speed);
    
    resetLeds();
    red = 0;
    green = 200;
    blue = 0;
    for (int i = 1; i <= LEDNUM / 2; i++){
      led.setColor(i, red, green, blue);
    }
    led.show();
  }
  firstTime = false;

  bool transition = dist < DISTANCE_SWITCH;
  if (transition){
    firstTime = true;
    currentState = SLOW;
  }
}


void slow(double dist){
  static bool firstTime = true;
  if (firstTime){
    speed = 100;
    digitalWrite(ML_IN2, LOW);
    digitalWrite(ML_IN1, HIGH);
    analogWrite(ML_PWM, speed);

    digitalWrite(MR_IN2, LOW);
    digitalWrite(MR_IN1, HIGH);
    analogWrite(MR_PWM, speed);
    
    resetLeds();
    red = 100;
    green = 100;
    blue = 0;
    for (int i = LEDNUM / 2; i <= LEDNUM; i++){
      led.setColor(i, red, green, blue);
    }
    led.show();
  }
  firstTime = false;

  bool transitionStop = dist < DISTANCE_LIMIT;
  if (transitionStop){
    firstTime = true;
    currentState = STOP;
  }
  	
  bool transitionDrive = dist >= DISTANCE_SWITCH;
  if (transitionDrive){
    firstTime = true;
    currentState = DRIVE;
  }
}

void stop(unsigned long ct){
  static bool firstTime = true;
  static unsigned long exitTime = 0;
  if (firstTime){
    speed = 0;
    analogWrite(ML_PWM, 0);
    analogWrite(MR_PWM, 0);

    resetLeds();
    red = 200;
    green = 0;
    blue = 0;
    led.setColor(red, green, blue);
    led.show();
    exitTime = ct + WAIT_TIME;
  }
  firstTime = false;

  bool transition = ct > exitTime;
  if (transition) {
    firstTime = true;
    currentState = BACKWARD;
  }
}

void backward(unsigned long ct){
  
}

