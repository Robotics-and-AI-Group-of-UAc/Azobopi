#ifndef main_h
#define main_h
#include <Arduino.h>
#include "ezButton.h"
//#include "NeoPixel/Adafruit_NeoPixel.h"
#include "ESP32MotorControl.h"
#include "PID_simple.h"

// debug
#define DEBUG
#ifdef DEBUG
# define DEBUG_PRINT(x) Serial.print(x)
# define DEBUG_PRINTLN(x) Serial.println(x)
#else // ifdef DEBUG
# define DEBUG_PRINT(x)
#endif // ifdef DEBUG

// Encoders pins
#define ENC1_A 34
#define ENC1_B 35
#define ENC2_A 36
#define ENC2_B 39

// Led NeoPixel - Comment first define if rgb led or three led version
#define NEOPIXEL 5
#ifdef NEOPIXEL
#include "NeoPixel/Adafruit_NeoPixel.h"
//# include <NeoPixel/Adafruit_NeoPixel.h>
# define NUMPIXELS 1
#else // ifdef NEOPIXEL

// Led pins
# define RED 5
# define GREEN 23
# define BLUE 22
#endif // ifdef NEOPIXEL

// Wheels
#define WHEEL_DIAMETER 66 // wheel diameter in mm
#define WHEEL_CIRCUMFERENCE (3.14 * WHEEL_DIAMETER)
#define WHEELS_DISTANCE 110
#define CURVE_CIRCUMFERENCE (3.14 * WHEELS_DISTANCE)

// States
#define VOID_ST 0
#define INIT_ST 1
#define START_EXEC_ST 2
#define EXEC_ST 3
#define READ_COMM_ST 4
#define STOP_ST 5
#define FORWARD_ST 6
#define TURN_RIGHT_ST 7
#define TURN_LEFT_ST 8
#define BACK_ST 9

// Movement Commands
#define MAX_NR_COMMANDS 20
#define STOP 0
#define TURN_LEFT 1
#define TURN_RIGHT 3
#define FORWARD 2
#define BACKWARD 4
#define ROTATION_TICKS 1920

// time motors are stopped
#define STOP_DELAY 500

// SetPoints for PID
#define SETPOINT_RUN 3900
#define SETPOINT_TURN 1540

unsigned long time_now;

// initial motor speed
int speedL = 40;
int speedR = 40;

// ezButton pins
ezButton button_yellow_left(18);
ezButton button_red(19);
ezButton button_yellow_bottom(21);
ezButton button_blue(17);
ezButton button_green(16);

// commands
int nr_comm;
int comm_index;         // the index of the action that is being executed...
int recorded_button[MAX_NR_COMMANDS];
int button_index = 0;
int mov;                // Programed data from buttons
unsigned long y_count;  // Nr. of times yellow left is pressed

int on_execute_test_st; // state control variable
int on_execute_comm_st; // state control variable

// TODO - state in movement
int machine_state;
int stop_next_state;

// PID
double val_outputL;
double val_outputR;
double enc_readL;
double enc_readR;
double Setpoint;
double kp = 0.0007, ki = 0.000008, kd = 8;
int    kspeed = 2;
volatile int counterPID;
int freq = 50;

// Encoders Interrupt function variables and table
volatile double encoder1_pos;
volatile double encoder2_pos;
byte encoder1_state, encoder2_state;
int  encoder_table[] = { 0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0 };

#endif // ifndef main_h
