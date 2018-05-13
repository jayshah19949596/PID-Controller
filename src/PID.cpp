#include "PID.h"
#include <vector>
#include <math.h>
#include <numeric>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd)
{

  tolerance = 0.005;
  dp = 0.1*Ki;
  dps = {1.0, 1.0, 1.0};

  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

}

void PID::UpdateError(double cte)
{
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;

  tot_error = this->TotalError(cte);
}

double PID::TotalError(double cte)
{
  return cte*cte;
}

void PID::Restart(uWS::WebSocket<uWS::SERVER> ws)
{
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}


void PID::PerformTwiddle(double total_error, double hyperparameter) {
  static double current_best_error = 100000;
  static bool is_twiddle_init = false;
  static bool is_twiddle_reset = false;
  static double last_hyperp = 0;
  int tolerance = 0.005;

  cout << "Current best error is: " << current_best_error << endl;
  cout << "Dp is: " << dp << endl;
  if (!is_twiddle_init) {
    cout << "Twiddle init";
    current_best_error = total_error;
    is_twiddle_init = true;
    return;
  }
  if ((fabs(dp) > tolerance)) {
    if (is_twiddle_reset) {
      cout << "Twiddle reset!-----------------------------" << endl;
      last_hyperp = hyperparameter;
      hyperparameter += dp;
      cout << "Hyperparameter magnitude increased!" << endl;
      is_twiddle_reset = false;
    } else {
      if (total_error < current_best_error) {
        dp *= 1.1;
        is_twiddle_reset = true;
        current_best_error = total_error;
      } else {
        if (fabs(last_hyperp) < fabs(hyperparameter)) {
          last_hyperp = hyperparameter;
          hyperparameter -= 2.0 * dp;
          cout << "Hyperparameter magnitude decreased!" << endl;
        } else {
          last_hyperp = hyperparameter;
          hyperparameter += dp;
          dp *= 0.9;
          cout << "Hyperparameter magnitude kept same!" << endl;
          is_twiddle_reset = true;
        }
      }
    }
  }
  cout << "Hyperparameter magnitude kept same! " << hyperparameter << endl;

}




