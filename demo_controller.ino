/*
   2DoF Differential Mechanism Demo

   Unit abbreviations used in function/variable declarations and comments:
   deg : degrees
   ms  : milliseconds
   us  : microseconds
   RPS : revolutions per seconds

   NOTES FOR TRANSFERING TO ARDUINO:
   - replace printf's with Serial.print/println's where needed
*/

#include <Servo.h>
#include <math.h>

/* Hardware Parameters */
#define SERVO_RANGE_DEG 270
#define NO_LOAD_SPD_RPS 1

// empirically determined servo pulse signal bandwidth bounds, us
#define SERVO_PULSE_MIN 600
#define SERVO_PULSE_MAX 2400
const int zero_offset = (SERVO_PULSE_MAX + SERVO_PULSE_MIN ) / 2;

#define DRIVE_GEAR_RATIO 1
#define DIFFERENTIAL_GEAR_RATIO 1

// The min ammount of time to delay impatient retargeting [ms].
// 0 retargets immediately, >0 continues previous move for a minimum of the perscribed time.
// Indended to reduce jitter if IMPATIENT_RETARGETING is used, or if sequential targets would cause rapid direction switching.
#define MIN_RETARGET_DELAY_MS 5

// externally togglable operating modes
typedef enum OP_MODE_SETTINGS
{
  ENABLE_MOVEMENTS = 1 << 0, // Enable/Disable exporting movements to motors. Does not affect other controller behaviours.
  FORCE_IK = 1 << 1, // Override any automatic switching between FK and IK for debugging and safe controller input handling.
  BUFFER_TARGETS = 1 << 2, // Add sequential positional commands to a buffer (currently 1), excecuted FIFO [TODO: implement]
  IMPATIENT_RETARGETING = 1 << 3, // Activate next target before current target is reached. Delay set by MIN_RETARGET_DELAY_MS
  LINEAR_INTERPOLATE_MOVES = 1 << 4, // calculates separate motor velocities so they complete movements in equal time
  AUTO_CONSTRAIN_TARGETS = 1 << 5, // Auto constrain positional commands.
  DEMO_MODE = 1 << 6, // controller generates targets automatically
  POT_CONTROL = 1 << 7 // enables potentiometer control for testing. TODO: implement
} OP_MODE_SETTINGS;

typedef enum COMMUNICATION_SETTINGS
{
  SUPPRESS_ALL = 1 << 0, // disables all (currently serial) reporting
  ECHO_CMDS = 1 << 1, // repeats back all TX
  ALERT_CONSTRAINED = 1 << 2, // reports constrained if AUTO_CONSTRAIN on, reports rejected if AUTO_CONSTRAIN off.
} COMMUNICATION_SETTINGS;

//
typedef enum TARGETING_FLAGS
{
  TARGET_BUFFERED = 1 << 0, // new target available in buffer (was NEW_PITCH_TARGE)
  TARGET_UPDATED = 1 << 1, // active target updated, IK needs recalculated
  SETPOINT_M0_REACHED = 1 << 2, // motor 0 commanded pos reached
  SETPOINT_M1_REACHED = 1 << 3, // motor 1 commanded pos reached
  PITCHYAW_POSITIONS_CURRENT = 1 << 4, // current pitch and yaw positions have been calculated this loop. prevents recalculating
  TARGET_REACHED = SETPOINT_M0_REACHED | SETPOINT_M1_REACHED // overall target reached. redundant by 3,4, but is cleaner
} TARGETING_FLAGS;

//-- Flags --//
volatile OP_MODE_SETTINGS opModeSettings;
volatile COMMUNICATION_SETTINGS reportingSettings;
volatile TARGETING_FLAGS targetStatus;

//-- Timekeeping --//
unsigned long time_prev_target_start = 0;

//-- Targeting Registers --//
float cur_pitch = 0;
float prev_target_pitch = 0;
float active_target_pitch = 0;
volatile float target_buffer_pitch; // TODO: re-implement as FIFO stack

float cur_yaw = 0;
float prev_target_yaw = 0;
float active_target_yaw = 0;
volatile float target_buffer_yaw = 0;

//-- Motor Position Registers --//
float setpoint_m0 = 0;
float setpoint_m1 = 0;
float pos_m0 = 0; // current position at a given time of m0
float pos_m1 = 0; // "  " for m1
int signal_m0 = 0;
int signal_m1 = 0;

//-- Speed --//
float base_speed = 0.5; // based on NO_LOAD_SPEED_RPS TODO: account for each loop time and add delay for constant speed
int step_delay = 1; // delay in ms after each movement increment. lower means faster movements
float speed_m0 = base_speed;
float speed_m1 = base_speed;

//-- physical constraints --//
float pitch_min = -90;
float pitch_max = 20; //actual max is 70, but a minor cable interference occurs
float yaw_min = -45;
float yaw_max = 45;


//-- motor objs --//
Servo m0;
Servo m1;

String inputString = ""; // gotta hold dem commands somewhere
String arg1, arg2; // these need to be globals for some reason, but only in this case. Likely due to how Arduino implemented it alongside AVRC

int checkTargetNeedsUpdate(void);

int checkTargetProximity(float pos, float setpoint, float speed);

void updateActiveTarget(void);

void computeMotorSetpoints(void);

void setSpeeds(void);

void updatePitchYawPositions(void);

void handleConstraining(float *target, float test_min, float test_max); // TODO: call this in serial handler

void handleConstrainingPulsewidth(int *target, int test_min, int test_max);

void resetLoopChecklist(void);

void adjustMotorPositionCmds(void);

void excecuteMotorTargets(void);

void serialEvent(void);

void handle_cmd(void);
// ----- -----//

void setup()
{
  // Opperational mode defaults
  // movements should not be enabled here, but in this case we need to tell the servos to zero as soon as they're attached
  opModeSettings = ENABLE_MOVEMENTS | BUFFER_TARGETS | IMPATIENT_RETARGETING | LINEAR_INTERPOLATE_MOVES;
  reportingSettings = ALERT_CONSTRAINED;

  inputString.reserve(200);
  Serial.begin(115200);

  m0.attach(9);
  m0.writeMicroseconds(zero_offset);
  m1.attach(10);
  m1.writeMicroseconds(zero_offset);

  Serial.println("--- Differential Position Mechanism Control Demo ---");
  Serial.println("Send \'h\' for available commands");
  //TODO: indicate which settings are preset
}

void loop()
{
  // TODO: function here that does automated/demo moves
  resetLoopChecklist();

  // Update pitch/yaw targets if needed
  if (checkTargetNeedsUpdate())
  {
    updateActiveTarget();
    computeMotorSetpoints();
    setSpeeds();
  }

  // update current pitch and yaw positions forward-kinematically
  updatePitchYawPositions();

  // increment motor positions by step ammount
  adjustMotorPositionCmds();

  signal_m0 = Angle2Pulse(pos_m0, 0);
  signal_m1 = Angle2Pulse(pos_m1, 1);

  //send off motor commands to motors
  excecuteMotorTargets();
  Serial.print("setpoint0: ");
  Serial.print(setpoint_m0);
  Serial.print(", pos_m1: ");
  Serial.println(pos_m1);
  delay(step_delay);
}

void excecuteMotorTargets(void)
{
  handleConstrainingPulsewidth(&signal_m0, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
  handleConstrainingPulsewidth(&signal_m1, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
  m0.writeMicroseconds(signal_m0);
  m1.writeMicroseconds(signal_m1);
}

int Angle2Pulse(float angle, int reverse)
{
  int pulsewidth;
  if (reverse)
  {
    pulsewidth = map(angle, -1 * (SERVO_RANGE_DEG / 2), SERVO_RANGE_DEG / 2, SERVO_PULSE_MAX, SERVO_PULSE_MIN);
  }
  else
  {
    pulsewidth = map(angle, -1 * (SERVO_RANGE_DEG / 2), SERVO_RANGE_DEG / 2, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
  }
  return pulsewidth;
}

void resetLoopChecklist(void)
{
  // resets stuff that needs to be every loop.
  targetStatus = ~(~targetStatus & PITCHYAW_POSITIONS_CURRENT);
}


int checkTargetNeedsUpdate(void)
{
  if (targetStatus & TARGET_BUFFERED)
  {
    if (millis() - time_prev_target_start >= MIN_RETARGET_DELAY_MS)
    {
      if (opModeSettings & IMPATIENT_RETARGETING)
      {
        updatePitchYawPositions();
        return 1;
      }
      else if (targetStatus & TARGET_REACHED)
      {
        // target reached, so save comp time and copy over.
        // TODO: should updatePITYawPositions() instead so dt across loops remains const?
        cur_pitch = active_target_pitch;
        cur_yaw = active_target_yaw;
        targetStatus |= PITCHYAW_POSITIONS_CURRENT;
        return 1;
      }
    }
  }
  return 0;
}


int checkTargetProximity(float pos, float setpoint, float speed)
{
  return (abs(pos - setpoint) <= speed); //TODO: replace fabs() with abs() in avrc (unless its small enough compiled)
}

void updateActiveTarget(void)
{
  prev_target_pitch = active_target_pitch;
  prev_target_yaw = active_target_yaw;

  // currently this just coppies from the single buffer vars and clears TARGET_BUFFERED
  active_target_pitch = target_buffer_pitch;
  active_target_yaw = target_buffer_yaw;

  targetStatus = ~(~targetStatus & TARGET_BUFFERED);
  // if a target stack/queue were introduced, it would deal with shifting the stack
  // it would only clear TARGET_BUFFERED if it had just reduced the stack to 0 elements.

  time_prev_target_start = millis();
  
  // this stays the same regardless
  targetStatus |= TARGET_UPDATED;
}

void computeMotorSetpoints(void)
{
  setpoint_m0 = (1 * active_target_yaw) + active_target_pitch;
  setpoint_m1 = (-1 * active_target_yaw) + active_target_pitch;

  targetStatus = ~(~targetStatus & TARGET_UPDATED); //TODO: re-write to check first, then set.
}


void setSpeeds(void)
{
  if ((opModeSettings & LINEAR_INTERPOLATE_MOVES) && () && ())
  {
    // TODO: try compiling with math.h to see if its too big. in retrospect it likely wont have a large affect
    // this way we don't need to load up any of math.h
    float d_pitch = cur_pitch - prev_target_pitch;
    d_pitch = d_pitch * d_pitch;
    Serial.println(d_pitch);
    float d_yaw = cur_yaw - prev_target_yaw;
    d_yaw = d_yaw * d_yaw;
    float dist = sqrt(d_pitch + d_yaw);

    float pitch_speed = (base_speed * sqrt(d_pitch)) / dist;
    float yaw_speed = (base_speed * sqrt(d_yaw)) / dist;
    speed_m0 = fabs((1 * yaw_speed) + pitch_speed);
    speed_m1 = fabs((-1 * yaw_speed) + pitch_speed);
  }
  else
  {
    speed_m0 = base_speed;
    speed_m1 = base_speed;
  }
}

void updatePitchYawPositions(void)
{
  if (targetStatus & PITCHYAW_POSITIONS_CURRENT)
  {
    // nothing for now, but might need later
  }
  else
  {
    cur_pitch = (pos_m0 + pos_m1) / 2;
    cur_yaw = (pos_m0 - pos_m1) / 2;
    targetStatus |= PITCHYAW_POSITIONS_CURRENT;
  }
}

void adjustMotorPositionCmds(void)
{
  if (checkTargetProximity(pos_m0, setpoint_m0, speed_m0))
  {
    pos_m0 = setpoint_m0;
    targetStatus |= SETPOINT_M0_REACHED;
  }
  else
  {
    if (pos_m0 - setpoint_m0 < 0)
    {
      pos_m0 += speed_m0;
    }
    else
    {
      pos_m0 -= speed_m0;
    }
  }
  if (checkTargetProximity(pos_m1, setpoint_m1, speed_m1))
  {
    pos_m1 = setpoint_m1;
    targetStatus |= SETPOINT_M1_REACHED;
  }
  else
  {
    if (pos_m1 - setpoint_m1 < 0)
    {
      pos_m1 += speed_m1;
    }
    else
    {
      pos_m1 -= speed_m1;
    }
  }
}


void handleConstraining(float *target, float test_min, float test_max)
{
  if ((*target < test_min) && (opModeSettings & AUTO_CONSTRAIN_TARGETS))
  {
    *target = test_min;
    if (reportingSettings & ALERT_CONSTRAINED)
    {
      Serial.println("Target constraint reached.");
      // TODO: print something better over serial
    }
  }
  else if (*target > test_max)
  {
    *target = test_max;
    if (reportingSettings & ALERT_CONSTRAINED)
    {
      Serial.println("Target constraint reached.");
    }
  }
}

void handleConstrainingPulsewidth(int *target, int test_min, int test_max)
{
  if ((*target < test_min) && (opModeSettings & AUTO_CONSTRAIN_TARGETS))
  {
    *target = test_min;
    if (reportingSettings & ALERT_CONSTRAINED)
    {
      Serial.println("Servo constraint reached.");
      // TODO: print something better over serial pulsewidth related
    }
  }
  else if (*target > test_max)
  {
    *target = test_max;
    if (reportingSettings & ALERT_CONSTRAINED)
    {
      Serial.println("Servo constraint reached.");
    }
  }
}

void handle_cmd() {
  /*
     A fairly simple command + argument protocol.
     Each command must be a single char (byte) so we can use a switch statement, which lets the compiler hash the commands for speed.
     Args may be of any type, and it is up to the individual command handler to deal with that
     This means you get 256 unique commands, though in practice you really only get the characters you can type, minus [Space] and [,], which we use as our delimeters.
     Reasonably usable characters are [ ! " # $ % & ' ( ) * + ` - . / : ; < = > ? @ [ \ ] ^ _ ` { | } ~ ], [0-9], [a-z], [A-Z]
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
      case 'p': target_buffer_pitch += (arg1.toFloat());
        targetStatus |= TARGET_BUFFERED;
        break;

      case 'y': target_buffer_yaw += (arg1.toFloat());
        targetStatus |= TARGET_BUFFERED;
        break;

      case 'P': target_buffer_pitch = (arg1.toFloat());
        targetStatus |= TARGET_BUFFERED;
        break;

      case 'Y': target_buffer_yaw = (arg1.toFloat());
        targetStatus |= TARGET_BUFFERED;
        break;

      // TODO implement l/L and r/R control

      case 'I': target_buffer_pitch = (arg1.toFloat()); // absolute simultaneous pitch/yaw move
        target_buffer_yaw = (arg2.toFloat());
        targetStatus |= TARGET_BUFFERED;
        break;

      case 's': speed_m0 = arg1.toFloat();
        break;

      case 'z': step_delay = arg1.toInt();
        break;

      case 'd': opModeSettings = ~(~opModeSettings & DEMO_MODE);
        break;

      case 'D': opModeSettings |= DEMO_MODE;
        break;

      case 'm': pitch_min = (arg1.toFloat());
        pitch_max = (arg2.toFloat());
        break;

      case 'M': yaw_min = (arg1.toFloat());
        yaw_max = (arg2.toFloat());
        break;

      case 'q': opModeSettings = ~(~opModeSettings & IMPATIENT_RETARGETING);
        break;

      case 'Q': opModeSettings |= IMPATIENT_RETARGETING;
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
