#include "Arduino.h"
#include "PID_simple.h"

PID::PID(double *Setpoint,
         double *EncRead,
         double *Output,
         double  kp,
         double  ki,
         double  kd) {
  _setpoint = Setpoint;
  _encread  = EncRead;
  _output   = Output;
  _kp       = kp;
  _ki       = ki;
  _kd       = kd;
}

bool PID::Compute() {
  double error, proportional, integral, derivative;

  error             = *_setpoint - abs(*_encread);
  proportional      = error;
  integral         += proportional;
  derivative        = proportional - last_proportional;
  last_proportional = proportional;
  *_output          = (proportional * _kp) + (integral * _ki) +
                      (derivative * _kd);
  dispP = proportional;
  dispI = integral;
  dispD = derivative;

  if (*_output) {
    return true;
  }
  else return false;
}

double PID::GetP() {
  return dispP;
}

double PID::GetI() {
  return dispI;
}

double PID::GetD() {
  return dispD;
}
