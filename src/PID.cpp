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
  dp = -0.01;
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


void PID::MyTwiddle(double total_error, vector<double> hyperparameters)
{
  static double current_best_error = 100000;
  static bool is_twiddle_init = false;
  static bool is_twiddle_reset = false;
  static vector<double> last_hyperp = {0, 0, 0};
  double sum_of_dps;

  sum_of_dps = std::accumulate(dps.begin(), dps.end(), 0);

  cout<<"Current best error is: "<< current_best_error<<endl;

  if (!is_twiddle_init)
  {
    cout<<"Twiddle init";
    current_best_error = total_error;
    is_twiddle_init = true;
  }


  for(int i=0 ; i<hyperparameters.size(); i++)
  {
    if (is_twiddle_reset)
    {
      cout<<"Twiddle reset!-----------------------------"<<endl;
      last_hyperp.at(i) = hyperparameters.at(i);
      hyperparameters.at(i) += dps.at(i);
      cout<<"Hyper-parameter magnitude increased!"<<endl;
      is_twiddle_reset = false;
    }
    else
    {
      if (total_error < current_best_error) {
        dps.at(i) *= 1.1;
        is_twiddle_reset = true;
        current_best_error = total_error;
      }
      else
      {
        if (fabs(last_hyperp.at(i)) < fabs(hyperparameters.at(i))) {
          last_hyperp.at(i) = hyperparameters.at(i);
          hyperparameters.at(i) -= 2.0 * dps.at(i);
          cout<<"Hyper-parameter magnitude decreased!"<<endl;
        } else {
          last_hyperp.at(i) = hyperparameters.at(i);
          hyperparameters.at(i) += dps.at(i);
          dps.at(i) *= 0.9;
          cout<<"Hyper-parameter magnitude kept same!"<<endl;
          is_twiddle_reset = true;
        }
      }
    }
  }

  cout << "hyper-parameter is "<< hyperparameters.at(0) << hyperparameters.at(1) << hyperparameters.at(2) << endl;
}


