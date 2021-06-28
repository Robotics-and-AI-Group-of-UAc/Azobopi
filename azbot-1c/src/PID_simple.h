#ifndef PID_simple_h
#define  PID_simple_h

class PID {
public:

  PID(int,
      double *,
      double *,
      double,
      double,
      double);
  bool Initialize( 
    double);
  bool   Compute();
  double GetP();
  double GetI();
  double GetD();
  double GetError();

private:

  double *_encread;
  double *_output;
  
  int _id;
  double _setpoint;

  double _kp;
  double _ki;
  double _kd;

  double dispP;
  double dispI;
  double dispD;
  double error;
  double proportional;
  double integral;
  double derivative;


  double last_proportional;
};

#endif // ifndef PID_simple_h
