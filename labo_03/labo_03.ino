#include <MeAuriga.h>
enum State {SETUP, SEG_A, PIVOT_A, SEG_B, PIVOT_B, SEG_C, PIVOT_C, END};
State currentState = SETUP;

const int PULSE = 9;
const double RATIO = 39.267;

const double DIAMETER = 6.5;
const double TICK_TOUR = PULSE * RATIO;
const double CIRC = PI * DIAMETER;
int toGo = 4 * 30;
const double NB_TICK = toGo/CIRC * TICK_TOUR;

const String DA = "2280684";
unsigned long currentTime = 0;


MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

void isr_process_encoder1(void)
{
  if(digitalRead(Encoder_1.getPortB()) == 0)
  {
    Encoder_1.pulsePosMinus();
  }
  else
  {
    Encoder_1.pulsePosPlus();;
  }
}

void isr_process_encoder2(void)
{
  if(digitalRead(Encoder_2.getPortB()) == 0)
  {
    Encoder_2.pulsePosMinus();
  }
  else
  {
    Encoder_2.pulsePosPlus();
  }
}

void setup() {
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  Serial.begin(115200);

  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  Encoder_1.setPulse(PULSE);
  Encoder_2.setPulse(PULSE);
  Encoder_1.setRatio(RATIO);
  Encoder_2.setRatio(RATIO);
  Encoder_1.setPosPid(1.8,0,1.2);
  Encoder_2.setPosPid(1.8,0,1.2);
  Encoder_1.setSpeedPid(0.18,0,0);
  Encoder_2.setSpeedPid(0.18,0,0);
}
void loop() {
  currentTime = millis();
  stateManager(currentTime);
}

void stateManager(unsigned long ct){
  switch (SETUP){
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