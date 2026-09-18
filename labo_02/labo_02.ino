#include <MeAuriga.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define LEDNUM  12
#define LEDPIN  44
#define SCREEN_HEIGHT 64
#define SCREEN_WIDTH 128
#define OLED_RESET -1

#define SCREEN_ADDRESS 0x3C 

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

enum RobotState {DRIVE, SLOW, STOP, BACKWARD, TURN};
RobotState currentState;

MeRGBLed led(PORT0, LEDNUM);
MeUltrasonicSensor distSensor(PORT_7);

const int MAX_SPEED = 255;
int driveSpeed = 0.8 * MAX_SPEED;

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
const int BACKWARD_TIME = 1000;
const int TURN_TIME = 1200;
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

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("Échec d'allocation SSD1306"));
    for(;;);
  }

  display.clearDisplay();
  
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

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Distance: ");
  display.print(dist);
  display.println(" cm");
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
    case TURN:
      turn(ct);
      break;
  }
}

void drive(double dist){
  static bool firstTime = true;
  if (firstTime){
    firstTime = false;
    
    speed = driveSpeed;
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
    led.setColor(LEDNUM, red, green, blue);
    led.show();
  }

  bool transition = dist < DISTANCE_SWITCH;
  if (transition){
    firstTime = true;
    currentState = SLOW;
  }
}


void slow(double dist){
  static bool firstTime = true;
  if (firstTime){
    firstTime = false;
    
    speed = driveSpeed / 2;
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
  static unsigned lastTime = 0;
  static unsigned long exitTime = 0;
  if (firstTime){
    firstTime = false;
    
    speed = 0;
    analogWrite(ML_PWM, speed);
    analogWrite(MR_PWM, speed);

    resetLeds();
    red = 200;
    green = 0;
    blue = 0;
    led.setColor(red, green, blue);
    led.show();
    exitTime = ct + WAIT_TIME;
  }

  bool transition = ct >= exitTime;
  if (transition) {
    firstTime = true;
    currentState = BACKWARD;
  }
}

void backward(unsigned long ct){
  static bool firstTime = true;
  static unsigned long exitTime = 0;
  
  if (firstTime){
    firstTime = false;

    speed = driveSpeed / 2;
    digitalWrite(ML_IN2, HIGH);
    digitalWrite(ML_IN1, LOW);
    analogWrite(ML_PWM, speed);

    digitalWrite(MR_IN2, HIGH);
    digitalWrite(MR_IN1, LOW);
    analogWrite(MR_PWM, speed);

    exitTime = ct + BACKWARD_TIME;
  }
  
  bool transition = ct >= exitTime;
  if (transition) {
    firstTime = true;
    currentState = TURN;
  }
}

void turn(unsigned long ct){
  static bool firstTime = true;
  static unsigned long exitTime = 0;

  if (firstTime){
    firstTime = false;
    
    speed = driveSpeed / 2;
    digitalWrite(ML_IN2, HIGH);
    digitalWrite(ML_IN1, LOW);
    analogWrite(ML_PWM, speed);

    digitalWrite(MR_IN2, LOW);
    digitalWrite(MR_IN1, HIGH);
    analogWrite(MR_PWM, speed);

    exitTime = ct + TURN_TIME;
  }

  bool transition = ct >= exitTime;
  if (transition){
    firstTime = true;
    currentState = DRIVE;
  }
}

