#include <vector>
#include <uWS/uWS.h>

#ifndef PID_H
#define PID_H

using namespace std;

class PID {
public:

  /*
 * Twiddle
 */
  float tolerance;
  double dp;
  vector<double> dps;

  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double tot_error;
  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError(double cte);

  /*
  * Perform Twiddle.
  */
  void PerformTwiddle(double total_error, double hyperparameter);

  /*
  * My Twiddle.
  */
  void MyTwiddle(double total_error, vector<double> hyperparameters);

  /*
  * Used to reset the simulator and bring the car back to first position.
  */
  void Restart(uWS::WebSocket<uWS::SERVER> ws);
};

#endif /* PID_H */
