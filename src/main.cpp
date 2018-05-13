#define _USE_MATH_DEFINES
#undef __STRICT_ANSI__
#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <cmath>

// for convenience
using json = nlohmann::json;
bool do_twiddle = false;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;
  PID pid_st, pid_sp, pid_th;
  // TODO: Initialize the pid variable.

  pid_st.Init(0.114843, 0.00000001292, 3.0447);
  //pid_th.Init(0.316731, 0.0000, 0.0226185);
  pid_st.Init(0.2, 0.00000001292, 3.0);


  h.onMessage([&pid_st, &pid_sp, &pid_th](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    static unsigned int timesteps = 0;
    static double total_error = 0.0;
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value, throttle_value;  // speed_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          pid_st.UpdateError(cte);
          steer_value = - pid_st.Kp * pid_st.p_error
                        - pid_st.Kd * pid_st.d_error
                        - pid_st.Ki * pid_st.i_error;

          /*pid_sp.UpdateError(fabs(cte));
          speed_value = 20 - pid_sp.Kp * pid_sp.p_error
                        - pid_sp.Kd * pid_sp.d_error
                        - pid_sp.Ki * pid_sp.i_error;*/

//          pid_th.UpdateError(fabs(cte));
//          throttle_value = 0.75 - pid_th.Kp * pid_th.p_error
//                                - pid_th.Kd * pid_th.d_error
//                                - pid_th.Ki * pid_th.i_error;

          if (do_twiddle)
          {
            if (timesteps > 1000)
            {
              //pid_st.MyTwiddle(total_error, pid_st.Ki);
              //vector <double> hp = {pid_st.Kp, pid_st.Ki, pid_st.Kd};
              pid_st.PerformTwiddle(total_error, pid_st.Ki);
              pid_st.Restart(ws);
              timesteps = 0;
              total_error = 0.0;
              return;
            }
            else
            {
              total_error += pow(cte, 2);
            }
            timesteps++;
          }

          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Throttle Value: " << throttle_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.4; // throttle_value;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen("127.0.0.1", port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
