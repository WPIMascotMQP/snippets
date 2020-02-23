#define STEP 25
#define DIR 26
#define ENA 24

#define NUM_STEPS 12800

#define PEAK_uS 200 
#define TROUGH_uS 50
#define DIR_DELAY_uS 50

unsigned long cur_time = 0;
unsigned long prev_time = 0;

int motor_positions[5];
int motor_setpoints[5];

bool stepLevelHigh = 0;

int driveStepper(int motor);
void doStep(int step_pin);

void setup() {
  Serial.begin(115200);
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ENA, OUTPUT);

}

void loop() {


}

int driveStepper(int motor) {

  if (motor_positions[motor] > motor_setpoints[motor]){
    digitalWrite(DIR, LOW);
    doStep(STEP);
    motor_positions[motor] -= 1;
    return 0;
  } else if (motor_positions[motor] < motor_setpoints[motor]){
    digitalWrite(STEP, HIGH);
    doStep(STEP);
    motor_positions[motor] += 1;
    return 0;
  } else {
    return 1;
  }
  
}

void doStep(int step_pin){
  // ccw 1, cw 0
  cur_time = ;
  if (stepLevelHigh){
    if (cur_time - prev_time >= PEAK_uS){
      digitalWrite(STEP, LOW);
      stepLevelHigh = 0;
      prev_time = cur_time;
    }
  }else if (!stepLevelHigh){
    if (cur_time - prev_time >= TROUGH_uS){
      digitalWrite(STEP, HIGH);
      stepLevelHigh = 1;
      prev_time = cur_time;
    }
}
