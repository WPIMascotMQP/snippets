#define STEP 25
#define DIR 26
#define ENA 24

#define NUM_STEPS 12800

#define PEAK_uS 200 
#define TROUGH_uS 50
#define DIR_DELAY_uS 50


bool ccw = true;

void setup() {
  Serial.begin(115200);
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ENA, OUTPUT);

}

void loop() {
  if (ccw == true) {
    digitalWrite(DIR, HIGH);
    ccw = false;
  } else {
    digitalWrite(DIR, LOW);
    ccw = true;
  }
  delayMicroseconds(DIR_DELAY_uS);

  for (int i = 0; i < NUM_STEPS; i++) {
    Serial.println(i);
    digitalWrite(STEP, HIGH);
    delayMicroseconds(PEAK_uS);
    digitalWrite(STEP, LOW);
    delayMicroseconds(TROUGH_uS);
  }

}
