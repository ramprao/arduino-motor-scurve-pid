/*
Program to move Motor in S curve with quadrature Encoder
Book: Trajectory Planning for Automatic Machines and Robots -- 
Authors: Luigi Biagiotti · Claudio Melchiorri
Section: 3.2.2 Trajectory with preassigned acceleration and velocity
Eqns: 3.8, 3.9, 3.10

Hardware -- 
1. Motor with encoder https://www.amazon.com/dp/B07GNGQ24C?th=1
2. Driver L293N https://www.amazon.com/dp/B0CLYBPGP9
3. Arduino Mega 2560
*/

#include <LibPrintf.h> //https://github.com/embeddedartistry/arduino-printf
#include <DCMotor.h>   //https://github.com/ArduinoSapienza/DCMotor
#include <Regexp.h>    //https://github.com/nickgammon/Regexp
#include <ArduPID.h>   //https://github.com/PowerBroker2/ArduPID

#define ENCODER_DO_NOT_USE_INTERRUPTS
// Stops motor and PID controller upon reaching DEADBAND
#ifndef  ENCODER_DO_NOT_USE_INTERRUPTS
float DEADBAND = 30;
#else
float DEADBAND = 10;
#endif

#include <Encoder.h> // https://github.com/PaulStoffregen/Encoder

#define BUFFER_LENGTH 30        //length of command to be sent over ethernet
#define TIMEOUT 2000
#define MIN_PWM_SPEED 40

// Some move commands
// pa 20000 moves 20000 counts absolute
// pr -123 moves -123 counts relative to current encoder reading
#define COM_MOVE_POS "^(p[ar]) ([abc]) (-?%d+)$"   // sets low pwmspeeds
#define COM_JOG "^jog ([abc]) (-?%d+) (%d+)$"   // sets low pwmspeeds
#define COM_STOP "^stop ([abc])$"   // sets low pwmspeeds
// Use if multiple motors/encoder/controllers
int8_t axis; // A= 1; B=2; C=3 etc

// Motors
DCMotor motorA = DCMotor(2, 4, 8); // 2,4 to IN1 and IN2 of L293N and 8 = PWM
DCMotor motorB = DCMotor(2, 4, 8); // 2,4 to IN1 and IN2 of L293N and 8 = PWM

//Encoder
Encoder EncA(21, 20); // Interrupts on Mega
Encoder EncB(19, 18); // Interrupts on Mega

//Define Variables we'll be connecting to for PID control
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
double KpA=1000.0, KiA=0, KdA=100.0; // 500, 1000, 13 // 1000, 0, 100//0.1,0.0,0.005
double KpB=0.1, KiB=0, KdB=0.005; // 500, 1000, 13 // 1000, 0, 100
ArduPID ControllerA;
ArduPID ControllerB;

// input buffer and RegExp match state for processing client input
char inBuffer[BUFFER_LENGTH];
int charsReceived = 0;
MatchState ms;

long ct2; // Use for printing in setup/loop

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  ct2 = millis();
  printf("Starting\n");
  //Setup the PID Loop
  axis = 0;
  setup_controllers();
}

void loop() {
  // put your main code here, to run repeatedly:

  // check to see if text received and then execute command
  if (Serial.available()) getReceivedTextSerial();

  // Keep printing encoder pos every second while waiting for input
  if (millis() - ct2 > 1000) {
  //printf("Waiting for input. Encoder = %f\n",Input);
  printf("%d %d\n", int(Setpoint), int(Setpoint) - enc_read(axis));
  ct2 = millis();
  }
}

void setup_controllers() {
  // Starting PID controller
  ControllerA.begin(&Input, &Output, &Setpoint, KpA, KiA, KdA);
  ControllerA.setSampleTime(1);      // OPTIONAL - will ensure at least 10ms have past between successful compute() calls
  ControllerA.setOutputLimits(-255, 255);

  ControllerB.begin(&Input, &Output, &Setpoint, KpB, KiB, KdB);
  ControllerB.setSampleTime(1);      // OPTIONAL - will ensure at least 10ms have past between successful compute() calls
  ControllerB.setOutputLimits(-255, 255);
}

void get_pos() 
{
  volatile float t, h;
  float newpos, prevOutput;
  float T, T_a;
  float q0, q1; // q0 = start_pos; q1 = end_pos
  volatile uint8_t absmove = 0;

  char *s = ms.GetCapture(inBuffer, 0);
  //printf("s = %s\n",s);
  if(strcmp(s,"pa") == 0) absmove = 1;

  axis = 0;
  char *ax = ms.GetCapture(inBuffer, 1);
  get_axis(ax, &axis);
  if (axis == 0) {
    printf("Invalid Axis\n");
    return;
  }

  newpos = float(atoi(ms.GetCapture(inBuffer, 2)));
  //printf("newpos = %f\n",newpos);
  q0 = float(enc_read(axis));
  if(absmove>0) {
    q1 = newpos;
  }
  else {
    q1 = q0 + newpos;
  }

  // First try. 
  do_pos(q0, q1);
  // Sometimes doesn't reach. So try again after delay.
  delay(100);
  q0 = float(enc_read(axis));
  if(abs(q1-q0) > 25) do_pos(q0, q1);

  // If not using interrupts it needs another try. 
  #ifdef  ENCODER_DO_NOT_USE_INTERRUPTS
  delay(100);
  q0 = float(enc_read(axis));
  if(abs(q1-q0) > 25) do_pos(q0, q1);
  #endif

}

// Determines whether trapezoid or triangle
uint8_t traj_type(float v_m, float a_m, float h) {
  if (h - (v_m/a_m)*(v_m/a_m) > 0) {
    return 1; // Trapezoid
  }
    else {
      return 0; // Triangle
    }
  }

// Determine Total and Acceleration time for the trajectory
void traj_time(float q0, float q1, float v_m, float a_m, float *T, float *T_a) {
  float h = abs(q1 - q0);
  uint8_t is_trap = traj_type(v_m, a_m, h);

  if (is_trap > 0) {
    *T = (h*a_m + v_m*v_m)/(a_m*v_m);
    *T_a = v_m/a_m;
  }
  else {
    *T = 2.0*sqrt(h/a_m);
    *T_a = *T/2.0;
  }
}

// Trajectory calculation
double pos_calc(float t, float q0, float q1, float T, float Ta, float v_m, float a_m) {
  float h = abs(q1 - q0);
  uint8_t is_trap = traj_type(v_m, a_m, h);
  int dir_move = (q1-q0)/abs(q1-q0);

  if (is_trap > 0) { // Trapezoid
    if ((t>=0) && (t<=Ta))
    {
      return float(q0 + dir_move*0.5*a_m*t*t); 
    }
    else if ((t>Ta) && (t<=T-Ta))
    {
      return float(q0 + dir_move*a_m * Ta * (t - Ta/2.0));
    }
    else
    {
      return float(q1 - dir_move*0.5 * a_m * (T-t)*(T-t));
    }
  }
  else { // Triangle
    if ((t>=0) && (t<=T/2.0))
    {
      return float(q0 + dir_move*0.5*a_m*t*t); 
    }
    else
    {
      return float(q1 - dir_move*0.5 * a_m * (T-t)*(T-t));
    }
  }
}

// Setup min outputs to MIN_PWM_SPEED
void set_minoutput(double *Output) {
  if ((*Output <0) && (*Output > -MIN_PWM_SPEED)) *Output = -MIN_PWM_SPEED; //-40
  if ((*Output >0) && (*Output < MIN_PWM_SPEED)) *Output = MIN_PWM_SPEED;     //40
}

// Moves to requested position via PID control
void do_pos(float q0, float q1) {

  // Max accel and max velosity for trajectory
  float a_m = 20000.0; // 10000 20000 20000
  float v_m = 8000.0; // 6000 7000 8000

  float T, T_a, t, prevOutput;
  float deltaT = 0.02; // 0.001 works; also 0.01
  //printf("q0 q1 = %f %f\n",q0, q1);
  float h = abs(q1 - q0);  
  uint8_t is_trap = traj_type(v_m, a_m, h);

  //printf("trap = %d\n", trap);

  controller_start(axis);

  // Computes T, Ta returned as pointers
  traj_time(q0, q1, v_m, a_m, &T, &T_a);

  //printf("T, T_a, deltaT = %f %f %f\n", T, T_a, deltaT);

  // Start S curve trajectory with PID
  long start_time = millis();
  while (millis() - start_time < 1000*T) {
    t = (millis() - start_time)/1000.0;
  
    Setpoint = pos_calc(t, q0, q1, T, T_a, v_m, a_m);

    long curr_time = millis();
    while ( millis() - curr_time < int(1000*deltaT)) {
      Input = float(enc_read(axis));
      controller_compute(axis);
      set_minoutput(&Output);
      if(prevOutput*Output < 0) {motor_off(axis);}; // stop if sign changes
      motor_on(axis, Output);
      prevOutput = Output;
    }
    Input = float(enc_read(axis));
  }
  // S curve is finished
  //printf("First round %f %f %f %f\n", t, Setpoint, Input, Output);

  //Final Point PID Loop 
  Input = float(enc_read(axis));
  long error = Input - Setpoint;
  long curr_time = millis();
  while ((millis() - curr_time < TIMEOUT)) { 
    controller_compute(axis);
    set_minoutput(&Output);    
    if(prevOutput*Output < 0) {motor_off(axis);}; // stop if sign changes
    motor_on(axis, Output);
    prevOutput = Output;
    Input = float(enc_read(axis));
    error = Input - Setpoint;
    if (abs(error) < DEADBAND) { 
      motor_off(axis) ;
      break;
    } 
  }

  motor_off(axis);
  delay(100);
  //printf("Elapsed time = %lu\n",millis()-start_time);
  Input = float(enc_read(axis));
  //if(abs(q1 - Input) < 20) {motor.off();}
  //printf("Final %f %f %f\n", t, Setpoint, Input);
  controller_stop(axis);
}

// Jogs for specified time at specified speed
void jog() {
  axis = 0;
  char *ax = ms.GetCapture(inBuffer, 0);
  get_axis(ax, &axis);
  if (axis == 0) {
    printf("Invalid Axis\n");
    return;
  }

  int speed = atoi(ms.GetCapture(inBuffer, 1));
  int tms = atoi(ms.GetCapture(inBuffer, 2));

  float stEnc = float(enc_read(axis));
  printf("speed, time, start_enc = %d, %d, %f\n",speed, tms, stEnc);
  motor_on(axis, speed);
  long start_time = millis();
  while (millis() - start_time < tms) {
   enc_read(axis);
  }
  motor_off(axis);
  delay(100);
  printf("speed, time, Difference Encoder = %d, %d, %f\n",speed, tms,float(enc_read(axis))-stEnc);
}

// Stops motor
void motor_stop() {
  axis = 0;
  char *ax = ms.GetCapture(inBuffer, 0);
  get_axis(ax, &axis);
  if (axis == 0) {
    printf("Invalid Axis\n");
    return;
  }
  //printf("Stopping motor\n");
  motor_off(axis);
}

void get_axis(char *ax, int8_t *axis) {
  *axis = 0;
  if(strcmp(ax,"a") == 0) *axis = 1;
  if(strcmp(ax,"b") == 0) *axis = 2;
}


float enc_read(int8_t axis) {
  switch (axis) {
    case 1:
      return float(EncA.read());
      break;
    case 2:
      return float(EncB.read());
      break;
    default:
      return 0.0;
  }
}

void motor_on(int8_t axis, double Speed) {
  switch (axis) {
    case 1:
      motorA.on(Speed);
      break;
    case 2:
      motorB.on(Speed);
      break;
    default:
      ;
  }
}

void motor_off(int8_t axis) {
  switch (axis) {
    case 1:
      motorA.off();
      break;
    case 2:
      motorB.off();
      break;
    default:
      ;
  }
}

void controller_start(int8_t axis) {
  switch (axis) {
    case 1:
      ControllerA.start();
      break;
    case 2:
      ControllerB.start();
      break;
    default:
      ;
  }
}

void controller_stop(int8_t axis) {
  switch (axis) {
    case 1:
      ControllerA.stop();
      break;
    case 2:
      ControllerB.stop();
      break;
    default:
      ;
  }
}

void controller_compute(int8_t axis) {
  switch (axis) {
    case 1:
      ControllerA.compute();
      break;
    case 2:
      ControllerB.compute();
      break;
    default:
      ;
  }
}


// From Paul Grimes -----------------------------------------------------------
// Gets command from Serial port
void getReceivedTextSerial()
{
  char c, d;
  int charsWaiting = 0;
  bool endOfText = false;
  
  // copy waiting characters into inBuffer
  // until inBuffer full, "\n" received, or no more characters
  charsWaiting = Serial.available();

  do {
    c = Serial.read();
    charsWaiting--;
    if (c == 0x0a) {
      // we've got a '\n'
      endOfText = true;
    }
    else if (c== 0x0d) {
      // we've got a '\r'
      ;
    }
    else {
      inBuffer[charsReceived] = c;
      charsReceived++;
    }
  }
  while(charsReceived <= BUFFER_LENGTH && endOfText && charsWaiting > 0);

  //if CR found go look at received text and execute command
  if(endOfText) {
    Serial.println("Got message:");
    Serial.println(inBuffer);
    parseReceivedText();
    clearBuffer();
  }

  // if textBuff full without reaching a CR, print an error message
  if(charsReceived >= BUFFER_LENGTH) {
    printErrorMessage();
    clearBuffer();
  }
  // if inBuffer not full and no CR, do nothing else;
  // go back to loop until more characters are received
}


//-----------------------------------------------------------
// Parse for command
void parseReceivedText()
{
  Serial.print("COMMAND ISSUED: ");
  Serial.println(inBuffer);

  ms.Target(inBuffer);
  if (ms.Match(COM_MOVE_POS) == 1) { get_pos(); }
  else if (ms.Match(COM_JOG) == 1) { jog(); }
  else if (ms.Match(COM_STOP) == 1) { motor_stop(); }
  else { printErrorMessage(); }
}

//-----------------------------------------------------------
// Clear char serial buffer
void clearBuffer()
{
  charsReceived = 0;
  memset(inBuffer, '\0', BUFFER_LENGTH);
}

//-----------------------------------------------------------
// Print error message
void printErrorMessage()
{
  Serial.print("ERROR: UNKNOWN COMMAND ");
  Serial.println(inBuffer);
}
