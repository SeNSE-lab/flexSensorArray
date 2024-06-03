#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <TimerOne.h>
#include <AccelStepper.h>

// Created by Kevin Kleczka May 2024, contact kevin.kleczka@northwestern.edu


////////// Initialize constants
// flag to denote command should be applied to all whiskers/column
#define ALL_FLAG 999

// Serial baud rate, make sure this matches with flexlib.BAUD_RATE
const unsigned long BAUD = 921600;

// Steps per revolution of motor. NOTE: since INTERLEAVE step mode is used, this is double the number of steps per revolution of the motor
const int STEPS_PER_REVOLUTION = 400;

//// hardware addresses
// These are used based on the integer value passed in
//    e.g. if no addr pins are soldered on a shield pass in 0, if the first and second from the right are soldered pass in 3
const uint8_t motorShield_addresses[] = { 0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D,
                                          0x6E, 0x6F, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x7B, 0x7C, 0x7D, 0x7E, 0x7F };
// these will be used in order as new whiskers and motors are added
//  e.g. if you set up 6 whiskers on 2 motors (3 whiskers each) the whisker signals will be read from A0 to A5, and limit pins will be 22 and 24
const int analogPins[] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15 };
const int limit_pins[] = { 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52 };


// Maximum supported hardware
const int MAX_SHIELDS = 16;
const int MAX_STEPPERS = 16;
const int MAX_WHISKERS = 16;

// Max number of steps allowed to be targeted
const long MAX_STEPS = 200;


////////// Initialize variables
// default step mode
int step_mode = INTERLEAVE;

// pointer arrays for motor control
Adafruit_MotorShield *motor_shields[MAX_SHIELDS];
Adafruit_StepperMotor *steppers[MAX_STEPPERS];
AccelStepper *accel_steppers[MAX_STEPPERS];

// default whisk steps
long default_prot = 133;
long default_ret = 22;

// default accelStepper maxSpeed and acceleration
long default_speed = 1500;
long default_accel = 500;

// set up column parameters
long prot_steps[MAX_STEPPERS] = { default_prot };
long ret_steps[MAX_STEPPERS] = { default_ret };
int is_left[MAX_STEPPERS] = { 0 };
int whisker_col_id[MAX_WHISKERS];
int whisking_status[MAX_STEPPERS] = { 0 };

// init number of connected components
int num_shields = 0;
int num_steppers = 0;
int num_whiskers = 0;

// datastream buffer
long output_buffer[7];
int buff_idx;

// sampling params
volatile int sampling = 0;
unsigned long sampling_period = 20000;

// allocate space for a few variables
int stepper_id;
int status;

// set debug messages for serial monitor debugging
int debug = 0;

//////// Accel stepper setup variables
// I gave up trying to figure out a better way to do this
// Forward and backward stepping functions for each stepper
void forwardStep0();
void backwardStep0();
void forwardStep1();
void backwardStep1();
void forwardStep2();
void backwardStep2();
void forwardStep3();
void backwardStep3();
void forwardStep4();
void backwardStep4();
void forwardStep5();
void backwardStep5();
void forwardStep6();
void backwardStep6();
void forwardStep7();
void backwardStep7();
void forwardStep8();
void backwardStep8();
void forwardStep9();
void backwardStep9();
void forwardStep10();
void backwardStep10();
void forwardStep11();
void backwardStep11();
void forwardStep12();
void backwardStep12();
void forwardStep13();
void backwardStep13();
void forwardStep14();
void backwardStep14();
void forwardStep15();
void backwardStep15();

// Function pointers for stepping functions
typedef void (*StepFunc)();

StepFunc forwardSteps[MAX_STEPPERS];
StepFunc backwardSteps[MAX_STEPPERS];

void initializeSteppers();


//////// Set up command communication protocol
// Number of words (argument variables) in a user command line.
char *argument_words[8];
// Buffer to use for receiving special user-defined ASCII commands.
char user_command_buffer[64];
// Index to use when filling the user command buffer.
uint8_t user_command_buffer_index = 0;
uint8_t c = 0;  // character received
char output_line[71];

void parse(char *line, char **argument_words, uint8_t maxArgs) {
  uint8_t argCount = 0;
  while (*line != '\0') { /* if not the end of line ....... */
    while (*line == ',' || *line == ' ' || *line == '\t' || *line == '\n')
      *line++ = '\0';         /* replace commas and white spaces with 0    */
    *argument_words++ = line; /* save the argument position     */
    argCount++;
    if (argCount == maxArgs - 1)
      break;
    while (*line != '\0' && *line != ',' && *line != ' ' && *line != '\t' && *line != '\n')
      line++; /* skip the argument until ...    */
  }
  *argument_words = '\0'; /* mark the end of argument list  */
}


// Timer1 interrupt service routine
void timerIsr() {
  // prints the current values over serial (ie sends character representations of all values)
  // prints a line for each attached whisker with values in following order:
  //  'timestamp', 'whisker_id', 'stepper_id', 'whisking_status', 'curr_steps', 'sensor_out'
  // NOTE that the millis() interrupt gets paused during handling of this intterrupt, so all whiskers will have the same timestamp
  if (sampling) {

    for (int i = 0; i < num_whiskers; i++) {
      buff_idx = 0;
      // 'timestamp
      output_buffer[buff_idx] = millis();
      buff_idx++;
      // whisker_id
      output_buffer[buff_idx] = i;
      buff_idx++;
      //stepper_id
      stepper_id = whisker_col_id[i];
      output_buffer[buff_idx] = stepper_id;
      buff_idx++;
      // whisking_status
      output_buffer[buff_idx] = whisking_status[stepper_id];
      buff_idx++;
      // curr_steps
      output_buffer[buff_idx] = accel_steppers[stepper_id]->currentPosition();
      buff_idx++;
      // sensor_out
      output_buffer[buff_idx] = analogRead(analogPins[i]);
      buff_idx++;
      for (int j = 0; j < buff_idx; j++) {
        Serial.print(output_buffer[j]);
        Serial.print(' ');
      }
      Serial.println();
    }
  }
}

void home_stepper(int stepper_id) {
  // retract stepper until limit switch is triggered
  while (digitalRead(limit_pins[stepper_id]) == HIGH) {
    if (is_left[stepper_id]) {
      steppers[stepper_id]->onestep(FORWARD, SINGLE);
    } else {
      steppers[stepper_id]->onestep(BACKWARD, SINGLE);
    }
    delay(25);
  }
  accel_steppers[stepper_id]->setCurrentPosition(0);
}



void setup() {

  Serial.begin(BAUD);

  delay(250);

  // let python know we're running
  Serial.println('!');

  // set reference voltage for analog read to 1.1 v to maximize signal range
  analogReference(INTERNAL1V1);

  //////// read setup commands
  // separate command and inputs with spaces ("cmd val1 val2 ...")

  // add shield with steppers and whiskers
  //  s shieldAddr steppersToAdd whiskersPerStepper sideColumnIsOn
  //    for last 3 params first digit is added first, second is added second
  //    e.g. "s 0 12 22 00" adds motor shield with addr 0, adds stepper 1 with 2 whiskers on the right side, then adds stepper 2 with 2 whiskers on the right side
  // set sampling interval in microseconds
  //  p interval
  //    e.g. "p 20000" sets the sample interval to 20,000 microseconds, or 50 Hz
  // complete setup
  //  c
  //    e.g. "c" denotes the setup is complete

  int setup_complete = 0;
  int sidx;
  int midx;
  int numw;
  int side = 0;
  num_whiskers = 0;
  while (!setup_complete) {
    while (Serial.available() > 0) {  // PC communication
      c = Serial.read();
      if (c == '\r') {
        user_command_buffer[user_command_buffer_index] = 0;
        parse((char *)user_command_buffer, argument_words, sizeof(argument_words));

        // add shield with steppers and whiskers
        //  s shieldAddr steppersToAdd whiskersPerStepper sideColumnIsOn
        if (strcmp(argument_words[0], "s") == 0) {
          sidx = atoi(argument_words[1]);
          midx = atoi(argument_words[2]);
          numw = atoi(argument_words[3]);
          side = atoi(argument_words[4]);

          int mone = midx % 10;
          int mten = (midx / 10) % 10;

          int wone = numw % 10;
          int wten = (numw / 10) % 10;

          int sone = side % 10;
          int sten = (side / 10) % 10;


          motor_shields[num_shields] = new Adafruit_MotorShield(motorShield_addresses[sidx]);
          motor_shields[num_shields]->begin();

          if (mten) {
            if (wten) {
              for (int i = 0; i < wten; i++) { whisker_col_id[i] = num_steppers; }
              num_whiskers += wten;
            }
            is_left[num_steppers] = sten;
            steppers[num_steppers] = motor_shields[num_shields]->getStepper(STEPS_PER_REVOLUTION, mten);
            num_steppers += 1;
          }
          if (mone) {
            if (wone) {
              for (int i = 0; i < wone; i++) { whisker_col_id[i] = num_steppers; }
              num_whiskers += wone;
            }
            is_left[num_steppers] = sone;
            steppers[num_steppers] = motor_shields[num_shields]->getStepper(STEPS_PER_REVOLUTION, mone);
            num_steppers += 1;
          }
          num_shields += 1;
        }
        // set sampling interval in microseconds
        //  p interval
        else if (strcmp(argument_words[0], "p") == 0) {
          sampling_period = atoi(argument_words[1]);
        }
        // complete setup
        //  c
        else if (strcmp(argument_words[0], "c") == 0) {
          setup_complete = 1;
        }

        user_command_buffer_index = 0;
      } else if (((c == '\b') || (c == 0x7f)) && (user_command_buffer_index > 0)) {
        user_command_buffer_index--;
      } else if ((c >= ' ') && (user_command_buffer_index < sizeof(user_command_buffer) - 1)) {
        user_command_buffer[user_command_buffer_index++] = c;
      }
    }
  }

  // initialize analog pins to read whisker values
  for (int i = 0; i < num_whiskers; i++) {
    pinMode(analogPins[i], INPUT);
  }
  // initialize limit pins for stepper homing
  for (int i = 0; i < num_steppers; i++) {
    pinMode(limit_pins[i], INPUT_PULLUP);
  }

  // initialize accelSteppers
  initializeSteppers();

  if (debug) Serial.println("setup complete");

  // initialize sampling timer intterupt
  Timer1.initialize(sampling_period);
  Timer1.attachInterrupt(timerIsr);
}

void loop() {

  //////// Read commands
  // separate command and inputs with spaces ("cmd val1 val2 ...")
  // Commands
  //  "set_prot" stepper_id prot_steps
  //  "set_ret" stepper_id ret_steps
  //  "whisk" stepper_id
  //  "set_speed" stepper_id speed
  //  "set_accel" stepper_id accel
  //  "set_sampling" bool
  //  "home_stepper" stepper_id
  //  "halt_whisk" stepper_id
  //  "retract" stepper_id

  if (Serial.available()) {
    while (Serial.available() > 0) {  // PC communication
      c = Serial.read();
      if (c == '\r') {
        user_command_buffer[user_command_buffer_index] = 0;
        parse((char *)user_command_buffer, argument_words, sizeof(argument_words));

        // set_prot stepper_id prot_steps
        if (strcmp(argument_words[0], "set_prot") == 0) {
          int sid = atoi(argument_words[1]);
          long steps = atoi(argument_words[2]);

          if (steps < MAX_STEPS && steps > 1) {

            if (sid == ALL_FLAG) {
              for (int i = 0; i < num_steppers; i++) {
                prot_steps[i] = steps;
              }
            } else {
              prot_steps[sid] = steps;
            }
          }
        }
        // set_ret stepper_id ret_steps
        else if (strcmp(argument_words[0], "set_ret") == 0) {
          int sid = atoi(argument_words[1]);
          long steps = atoi(argument_words[2]);

          if (steps < MAX_STEPS && steps > 1) {

            if (sid == ALL_FLAG) {
              for (int i = 0; i < num_steppers; i++) {
                ret_steps[i] = steps;
              }
            } else {
              ret_steps[sid] = steps;
            }
          }
        }
        // set_speed stepper_id speed
        else if (strcmp(argument_words[0], "set_speed") == 0) {
          int sid = atoi(argument_words[1]);
          long speed = atoi(argument_words[2]);

          if (sid == ALL_FLAG) {
            for (int i = 0; i < num_steppers; i++) {
              accel_steppers[i]->setMaxSpeed(speed);
            }
          } else {
            accel_steppers[sid]->setMaxSpeed(speed);
          }
        }
        // set_accel stepper_id accel
        else if (strcmp(argument_words[0], "set_accel") == 0) {
          int sid = atoi(argument_words[1]);
          long accel = atoi(argument_words[2]);

          if (sid == ALL_FLAG) {
            for (int i = 0; i < num_steppers; i++) {
              accel_steppers[i]->setAcceleration(accel);
            }
          } else {
            accel_steppers[sid]->setAcceleration(accel);
          }
        }
        // whisk stepper_id
        else if (strcmp(argument_words[0], "whisk") == 0) {
          int sid = atoi(argument_words[1]);

          if (sid == ALL_FLAG) {
            for (int i = 0; i < num_steppers; i++) {
              whisking_status[i] = 2;
              accel_steppers[i]->moveTo(prot_steps[i]);
            }
          } else {
            whisking_status[sid] = 2;
            accel_steppers[sid]->moveTo(prot_steps[sid]);
          }
        }
        // set_sampling bool
        else if (strcmp(argument_words[0], "set_sampling") == 0) {
          sampling = atoi(argument_words[1]);
        }
        // home_stepper stepper_id
        else if (strcmp(argument_words[0], "home_stepper") == 0) {
          int sid = atoi(argument_words[1]);

          if (sid == ALL_FLAG) {
            for (int i = 0; i < num_steppers; i++) {
              home_stepper(i);
            }
          } else {
            home_stepper(sid);
          }
        }
        // halt_whisk stepper_id
        else if (strcmp(argument_words[0], "halt_whisk") == 0) {
          int sid = atoi(argument_words[1]);

          if (sid == ALL_FLAG) {
            for (int i = 0; i < num_steppers; i++) {
              accel_steppers[i]->stop();
              // accel_steppers[i]->moveTo(accel_steppers[i]->currentPosition());
              whisking_status[i] = 1;
            }
          } else {
            accel_steppers[sid]->stop();
            // accel_steppers[sid]->moveTo(accel_steppers[sid]->currentPosition());
            whisking_status[sid] = 1;
          }
        }
        // retract stepper_id
        else if (strcmp(argument_words[0], "retract") == 0) {
          int sid = atoi(argument_words[1]);

          if (sid == ALL_FLAG) {
            for (int i = 0; i < num_steppers; i++) {
              accel_steppers[i]->moveTo(ret_steps[i]);
              whisking_status[i] = 1;
            }
          } else {
            accel_steppers[sid]->moveTo(ret_steps[sid]);
            whisking_status[sid] = 1;
          }
        }

        user_command_buffer_index = 0;
      } else if (((c == '\b') || (c == 0x7f)) && (user_command_buffer_index > 0)) {
        user_command_buffer_index--;
      } else if ((c >= ' ') && (user_command_buffer_index < sizeof(user_command_buffer) - 1)) {
        user_command_buffer[user_command_buffer_index++] = c;
      }
    }
  }

  // Check if protraction or retraction is complete, sets to next status if previous status is complete, run the motor
  for (int i = 0; i < num_steppers; i++) {
    if (whisking_status[i]) {
      if (accel_steppers[i]->distanceToGo() == 0) {
        whisking_status[i]--;
        if (whisking_status[i]) {
          accel_steppers[i]->moveTo(ret_steps[i]);
        }
      }
    }
    accel_steppers[i]->run();
  }
}


// initialize accelSteppers once setup is complete

void forwardStep0() {
  steppers[0]->onestep(FORWARD, INTERLEAVE);
}
void backwardStep0() {
  steppers[0]->onestep(BACKWARD, INTERLEAVE);
}
void forwardStep1() {
  steppers[1]->onestep(FORWARD, INTERLEAVE);
}
void backwardStep1() {
  steppers[1]->onestep(BACKWARD, INTERLEAVE);
}
void forwardStep2() {
  steppers[2]->onestep(FORWARD, INTERLEAVE);
}
void backwardStep2() {
  steppers[2]->onestep(BACKWARD, INTERLEAVE);
}
void forwardStep3() {
  steppers[3]->onestep(FORWARD, INTERLEAVE);
}
void backwardStep3() {
  steppers[3]->onestep(BACKWARD, INTERLEAVE);
}
void forwardStep4() {
  steppers[4]->onestep(FORWARD, INTERLEAVE);
}
void backwardStep4() {
  steppers[4]->onestep(BACKWARD, INTERLEAVE);
}
void forwardStep5() {
  steppers[5]->onestep(FORWARD, INTERLEAVE);
}
void backwardStep5() {
  steppers[5]->onestep(BACKWARD, INTERLEAVE);
}
void forwardStep6() {
  steppers[6]->onestep(FORWARD, INTERLEAVE);
}
void backwardStep6() {
  steppers[6]->onestep(BACKWARD, INTERLEAVE);
}
void forwardStep7() {
  steppers[7]->onestep(FORWARD, INTERLEAVE);
}
void backwardStep7() {
  steppers[7]->onestep(BACKWARD, INTERLEAVE);
}
void forwardStep8() {
  steppers[8]->onestep(FORWARD, INTERLEAVE);
}
void backwardStep8() {
  steppers[8]->onestep(BACKWARD, INTERLEAVE);
}
void forwardStep9() {
  steppers[9]->onestep(FORWARD, INTERLEAVE);
}
void backwardStep9() {
  steppers[9]->onestep(BACKWARD, INTERLEAVE);
}
void forwardStep10() {
  steppers[10]->onestep(FORWARD, INTERLEAVE);
}
void backwardStep10() {
  steppers[10]->onestep(BACKWARD, INTERLEAVE);
}
void forwardStep11() {
  steppers[11]->onestep(FORWARD, INTERLEAVE);
}
void backwardStep11() {
  steppers[11]->onestep(BACKWARD, INTERLEAVE);
}
void forwardStep12() {
  steppers[12]->onestep(FORWARD, INTERLEAVE);
}
void backwardStep12() {
  steppers[12]->onestep(BACKWARD, INTERLEAVE);
}
void forwardStep13() {
  steppers[13]->onestep(FORWARD, INTERLEAVE);
}
void backwardStep13() {
  steppers[13]->onestep(BACKWARD, INTERLEAVE);
}
void forwardStep14() {
  steppers[14]->onestep(FORWARD, INTERLEAVE);
}
void backwardStep14() {
  steppers[14]->onestep(BACKWARD, INTERLEAVE);
}
void forwardStep15() {
  steppers[15]->onestep(FORWARD, INTERLEAVE);
}
void backwardStep15() {
  steppers[15]->onestep(BACKWARD, INTERLEAVE);
}

void initializeSteppers() {
  // for (int i = 0; i < num_steppers; i++) {
  //     steppers[i] = AFMS.getStepper(200, i + 1);  // Steppers on ports 1 to 16
  // }

  forwardSteps[0] = forwardStep0;
  backwardSteps[0] = backwardStep0;
  forwardSteps[1] = forwardStep1;
  backwardSteps[1] = backwardStep1;
  forwardSteps[2] = forwardStep2;
  backwardSteps[2] = backwardStep2;
  forwardSteps[3] = forwardStep3;
  backwardSteps[3] = backwardStep3;
  forwardSteps[4] = forwardStep4;
  backwardSteps[4] = backwardStep4;
  forwardSteps[5] = forwardStep5;
  backwardSteps[5] = backwardStep5;
  forwardSteps[6] = forwardStep6;
  backwardSteps[6] = backwardStep6;
  forwardSteps[7] = forwardStep7;
  backwardSteps[7] = backwardStep7;
  forwardSteps[8] = forwardStep8;
  backwardSteps[8] = backwardStep8;
  forwardSteps[9] = forwardStep9;
  backwardSteps[9] = backwardStep9;
  forwardSteps[10] = forwardStep10;
  backwardSteps[10] = backwardStep10;
  forwardSteps[11] = forwardStep11;
  backwardSteps[11] = backwardStep11;
  forwardSteps[12] = forwardStep12;
  backwardSteps[12] = backwardStep12;
  forwardSteps[13] = forwardStep13;
  backwardSteps[13] = backwardStep13;
  forwardSteps[14] = forwardStep14;
  backwardSteps[14] = backwardStep14;
  forwardSteps[15] = forwardStep15;
  backwardSteps[15] = backwardStep15;

  for (int i = 0; i < num_steppers; i++) {
    accel_steppers[i] = new AccelStepper(forwardSteps[i], backwardSteps[i]);
    accel_steppers[i]->setMaxSpeed(default_speed);
    accel_steppers[i]->setAcceleration(default_accel);
  }
}
