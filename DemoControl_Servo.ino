/* 
 * Servo position calculations are done directly in pulse widths [int] rather than angles [float] for SPEED
 * 
 * Kinematic Mode: 
 *    When using serial control, the kinematic mode switches automatically depending on which positional commands are used. 
 *     
 */

#include <Servo.h>
#include <stdint.h>

#define SERVO_RANGE_DEG 270
#define SERVO_PULSE_MIN 600
#define SERVO_PULSE_MAX 2400

#define DRIVE_GEAR_RATIO 1
#define DIFFERENTIAL_GEAR_RATIO 1

Servo m0;
Servo m1;

String inputString = ""; // gotta hold dem commands somewhere
String arg1, arg2; // these need to be globals for some reason, but only in this case. Likely due to how Arduino implemented it alongside AVRC

bool zero_at_mid = 1; // offset motor/output rotation so zero is at half articulation range

bool pot_ctrl = 0; // 0: positional commands from serial
                   // 1: positional commands from potentiometers
                   
bool use_ik = 1; // 0: forward kinematic mode. l/L, r/R, pots control motors. p/P, y/Y trim output.
                 // 1: inverse kinematic mode. p/P, y/Y, pots control output. l/L, r/R trim motors.

bool buffer_movements = 0; // 0: a new setpoint overrides the current setpoint
                           // 1: wait to finish current movement, then start next movement
                           // NOT IMPLEMENTED

const int zero_offset = (SERVO_PULSE_MAX + SERVO_PULSE_MIN ) / 2;

bool demo_mode = 0;

// degrees to increment per loop. lower means more granular movements
float speed_m0 = 0.2;
float speed_m1 = speed_m0;

int step_delay = 1; // delay in ms after each movement increment. lower means faster movements

float pos_m0 = 0; //current motor rotation angles, in pulsewidth.
float pos_m1 = 0;
float prev_m0 = 0;
float prev_m1 = 0;

int m0_settled = 0;
int m1_settled = 0;


float setpoint_m0 = 0; //motor setpoint rotation angles, in deg.
float setpoint_m1 = 0; // currently used for both input and output depending on mode. yup, ew
float last_cmd_m0 = 0;
float last_cmd_m1 = 0;

float setpoint_pitch = 0; //output setpoint rotation angles, in deg.
float setpoint_yaw = 0; // currently used for both input and output depending on mode. yup, ew


// max output angles, in deg. for now use these as a starting point
float pitch_min = -90;
float pitch_max = 20; // actual max is 70, but a minor pulley collision occurs
float yaw_min = -45; // no actual limit yaw, just for demo mode
float yaw_max = 45; // ditto ^


int Angle2Pulse(float angle);

//void serialEvent(void); // we'll hijack built in serial behavior so we can have fancy commands with args
                        // doing it this way lets us piggyback onto the existing UART interupt vector
void handle_cmd(void);  // the actual func to parse and 'serve' the commands note: requires serial line ending to be \n

void setup() {
  inputString.reserve(200); // it would suck to not have enough RAM to store our commands
  Serial.begin(115200);
  
//  if ((zero_at_mid)) {
//    pos_m0 = zero_offset; // instantaneous positional command angles
//    pos_m1 = zero_offset; 
//  }
  m0.attach(9); 
  m0.writeMicroseconds(pos_m0);
  m1.attach(10);
  m1.writeMicroseconds(pos_m1);

  Serial.println("--- Differential Position Mechanism Control Demo ---");
  Serial.println("Send \'h\' for available commands");

}

void loop() {

  //TODO: implement POT control
  //TODO: interpolate between positions for smooth motion

  if ((demo_mode == 1) && (m0_settled == 1) && (m1_settled == 1)) {
    setpoint_pitch = random(pitch_min, pitch_max);//.toFloat();
    setpoint_yaw = random(yaw_min, yaw_max);//.toFloat();
    m0_settled = 0;
    m1_settled = 0;
    calcSpeeds();
  }
  
  if ((use_ik)) {
    setpoint_pitch = constrain(setpoint_pitch, pitch_min, pitch_max);
    setpoint_yaw = constrain(setpoint_yaw, yaw_min, yaw_max);
   
    // first apply the yaw
    setpoint_m0 = 1 * setpoint_yaw;
    setpoint_m1 = -1 * setpoint_yaw;
    // then apply the pitch
    setpoint_m0 += setpoint_pitch;
    setpoint_m1 += setpoint_pitch;
    calcSpeeds();
  }

  if (abs(pos_m0 - setpoint_m0) <= speed_m0) {
    pos_m0 = setpoint_m0;
    m0_settled = 1;

  } else {
    if ((pos_m0 - setpoint_m0 < 0)) {
      pos_m0 += speed_m0;
    } else {
      pos_m0 -= speed_m0;
    } 
  }

  if ((abs(pos_m1 - setpoint_m1) <= speed_m1)) {
    pos_m1 = setpoint_m1;
    m1_settled = 1;
  } else {
    if ((pos_m1 - setpoint_m1 < 0)) {
      pos_m1 += speed_m1;
    } else{
      pos_m1 -= speed_m1;
    } 
  }

  Serial.print("pitch: ");
  Serial.print(setpoint_pitch);
  Serial.print(", yaw: ");
  Serial.print(setpoint_yaw);
  Serial.print(", m0: ");
  Serial.print(pos_m0);
  Serial.print(", m1: ");
  Serial.println(pos_m1);
  
  m0.writeMicroseconds(constrain(Angle2Pulse(pos_m0), SERVO_PULSE_MIN, SERVO_PULSE_MAX));
  m1.writeMicroseconds(constrain(Angle2Pulserev(pos_m1), SERVO_PULSE_MIN, SERVO_PULSE_MAX));

  delay(step_delay);
}


int Angle2Pulse(float angle) {

  // TODO: add case where zero_at_mid is false
  // TODO: add option to reverse output. 
  int pulsewidth = map(angle, -1*(SERVO_RANGE_DEG / 2), SERVO_RANGE_DEG / 2, SERVO_PULSE_MIN, SERVO_PULSE_MAX);

  return pulsewidth;
}

int Angle2Pulserev(float angle) {

  // TODO: add case where zero_at_mid is false
  // TODO: add option to reverse output. 
  int pulsewidth = map(angle, -1*(SERVO_RANGE_DEG / 2), SERVO_RANGE_DEG / 2, SERVO_PULSE_MAX, SERVO_PULSE_MIN);

  return pulsewidth;
}

void calcSpeeds(void) {
  if (pos_m0 - prev_m0 != 0){
      speed_m1 = ((speed_m0 * abs(setpoint_m1 - prev_m1)) / abs(setpoint_m0 - prev_m0)) + 0.01;
    speed_m1 = speed_m0 + 0.01; // quick bodge so we don't get a inf error, same above
  }
}




void handle_cmd() {
  /*
   * A fairly simple command + argument protocol. 
   * Each command must be a single char (byte) so we can use a switch statement, which lets the compiler hash the commands for speed.
   * Args may be of any type, and it is up to the individual command handler to deal with that
   * This means you get 256 unique commands, though in practice you really only get the characters you can type, minus [Space] and [,], which we use as our delimeters.
   * Reasonably usable characters are [ ! " # $ % & ' ( ) * + ` - . / : ; < = > ? @ [ \ ] ^ _ ` { | } ~ ], [0-9], [a-z], [A-Z]
   */
  inputString.trim(); // removes beginning and ending white spaces
  char cmd = inputString.charAt(0); // for comprehension: cmd is technically arg0
  
  if ((cmd)) {
   
    // Be warned, this is a VERY poor and inefficient way to implement multiple args, but we only need max 2, and this is quick  
    int cur  = inputString.indexOf(' ');
    int cur1 = inputString.indexOf(',');
   
    if ((cur1 != -1)) {  
      arg1 = inputString.substring(cur + 1, cur1);
      arg2 = inputString.substring(cur1 + 1);  
    } else {
      arg1 = inputString.substring(cur + 1); 
      arg2 = "";
    }
    
    switch (cmd) {
      case 'p': use_ik = 1; // inc/dec pitch
                prev_m0 = pos_m0;
                prev_m1 = pos_m1;
                setpoint_pitch += (arg1.toFloat());
                calcSpeeds();
                m0_settled = 0;
                m1_settled = 0;
                break;
                
      case 'y': use_ik = 1; // inc/dec yaw
                prev_m0 = pos_m0;
                prev_m1 = pos_m1;
                setpoint_yaw += (arg1.toFloat());
                calcSpeeds();
                m0_settled = 0;
                m1_settled = 0;
                break;
                
      case 'P': use_ik = 1; // set abs pitch
                prev_m0 = pos_m0;
                prev_m1 = pos_m1;
                setpoint_pitch = (arg1.toFloat());
                calcSpeeds();
                m0_settled = 0;
                m1_settled = 0;
                break;
                
      case 'Y': use_ik = 1; // set abs yaw
                prev_m0 = pos_m0;
                prev_m1 = pos_m1;
                setpoint_yaw = (arg1.toFloat());
                calcSpeeds();
                m0_settled = 0;
                m1_settled = 0;
                break;

      // TODO implement l/L and r/R control
                
      case 'I': use_ik = 1; // absolute simultaneous pitch/yaw move
                prev_m0 = pos_m0;
                prev_m1 = pos_m1;
                setpoint_pitch = (arg1.toFloat());
                setpoint_yaw = (arg2.toFloat());
                calcSpeeds();
                m0_settled = 0;
                m1_settled = 0;
                break;

//      case 'F': use_ik = 0; // absolute simultaneous m1/m2 move 
//                break;

      case 's': speed_m0 = arg1.toFloat(); 
                break;

      case 'z': step_delay = arg1.toInt();
                break;
      case 'd' : demo_mode = 0;
                break;
      case 'D' : demo_mode = 1;
                break;

      case 'm': pitch_min = (arg1.toFloat());
                pitch_max = (arg2.toFloat());
                break;

      case 'M': yaw_min = (arg1.toFloat());
                yaw_max = (arg2.toFloat());
                break;

      case 'q': buffer_movements = 0;
                break;

      case 'Q': buffer_movements = 1;
                break;

      default: Serial.print("Unknown command \'"); 
               Serial.print(cmd);
               Serial.print("\'. Use \'h\' for a list of valid commands.");
               break;
    }
    inputString = "";
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      handle_cmd();
    }
  }
}
