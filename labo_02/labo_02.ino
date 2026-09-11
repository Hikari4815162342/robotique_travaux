#include <MeAuriga.h>

#define LEDNUM  12
#define LEDPIN  44

enum RobotState {DRIVE, SLOW, STOP, BACKWARD, TURN};
RobotState currentState = DRIVE;

MeRGBLed led(PORT0, LEDNUM);
MeUltrasonicSensor distSensor(PORT_7);

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
}

void loop() {
  currentTime = millis();
  distance = getDistance(currentTime);
  distancePrintTask(currentTime, distance);
}

double getDistance(unsigned long ct){
  static unsigned long lastTime = 0;
  int rate = 100;
  static double lastDistance = 0;

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
    default:
      stop(ct);
      break;
  }
}

void drive(double distance){
  static bool firstTime = true;
  int driveSpeed = 200;
  if (firstTime){
    digitalWrite(ML_IN2, HIGH);
    digitalWrite(ML_IN1, LOW);
    analogWrite(ML_PWM, driveSpeed);

    digitalWrite(MR_IN2, HIGH);
    digitalWrite(MR_IN1, LOW);
    analogWrite(MR_PWM, driveSpeed);
    
    red = 0;
    green = 200;
    blue = 0;
    for (int i = 1; i <= LEDNUM / 2; i++){
      led.setColor(i, 0, 200, 0);
    }
    firstTime = false;
  }
}

void slow(double distance){

}

void stop(unsigned long ct){

}
