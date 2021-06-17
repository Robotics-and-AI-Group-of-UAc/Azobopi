#ifndef PID_simple_h
#define  PID_simple_h

class PID {
public:

  PID(double *,
      double *,
      double *,
      double,
      double,
      double);

  bool   Compute();
  double GetP();
  double GetI();
  double GetD();

private:

  double *_setpoint;
  double *_encread;
  double *_output;

  double _kp;
  double _ki;
  double _kd;

  double dispP;
  double dispI;
  double dispD;

  double last_proportional;
};

#endif // ifndef PID_simple_h
