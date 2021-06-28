#include "Arduino.h"
#include "PID_simple.h"

PID::PID(int id,
         double *encRead,
         double *output,
         double  kp,
         double  ki,
         double  kd) {
  _id = id;
  _setpoint = 0;
  _encread  = encRead;
  _output   = output;
  _kp       = kp;
  _ki       = ki;
  _kd       = kd;
  // Initializing values
  integral = 0;
  derivative = 0;
  proportional =0;

}

bool PID::Initialize(double setpoint){
  _setpoint = setpoint;
  integral = 0;
  derivative = 0;
  proportional = 0;
  return true;
}


bool PID::Compute() {
  error             = _setpoint - abs(*_encread);
  proportional      = error;
  integral         += proportional;
  derivative        = proportional - last_proportional;
  last_proportional = proportional;
  *_output          = (proportional * _kp) + (integral * _ki) +
                      (derivative * _kd);

  //Degug
  /*
  Serial.print("ID:");
  Serial.print(_id);
  Serial.print(" Error:");
  Serial.print(error);
  Serial.println();
  */
  if (*_output) {
    return true;
  }
  else return false;
}

double PID::GetP() {
  return proportional;
}

double PID::GetI() {
  return integral;
}

double PID::GetD() {
  return derivative;
}
double PID::GetError(){
  return error;
}
