#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

const double Lf = 2.67;             // Given: this is the length from front to CoG that has a similar radius.

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          
          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];     



          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          
          /***** MY CODE *****/

          /*****************

          NOTE: This code was based on class material in the class quizzes, plus the YouTube overview. 

          Objectives:
          (1) Speed: if cost function sufficiently minimized, increase speed
          (2) Follow planned trajectory: minmize error (Cross Track Error and Psi error)
          from trajectory center line.

          ptsx/ptsy --> upcoming waypoints: 6 waypoints provided by sim
          x/y --> position of the car
          psi --> angle of car
          speed --> speed of car
          
          YouTube vid overview:
          https://www.youtube.com/watch?v=bOQuhpz3YfU&index=5&list=PLAwxTw4SYaPnfR7TzRZN-uxlxGbqxhtm2

          ******************/

          // Model actuator latency into system:
          // Identify where car will be after latency, prior to passing to MPC
          // (See sleep command below to insert 100 ms delay)

          double dt = 0.1;                          // 100 ms latency

          double x1 = px + v * cos(psi) * dt;       // Motion equations from MPC.cpp 
          double y1 = py + v * sin(psi) * dt;  
          double psi1 = psi - v * steer_value / Lf * dt;   
          double v1 = v + throttle_value * dt;      // Throttle value is proxy for acceleration
          
          px = x1;                                  // Update variables with new delayed values
          py = y1;
          psi = psi1;
          v = v1;

          // Shift car to origin and reference angle to make psi zero
          // Makes polyfit easier since car in some orientation to intended trajectory, which are both horizontal. 
          // Horizontal orientation preferred b/c polyfit may not be a function at vertical.
          for (int i=0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i]-px;    // sub all points from current position to shift to coord (0,0)
            double shift_y = ptsy[i]-py;
            
            // rotate cw to make psi 0
            ptsx[i] = (shift_x*cos(0-psi)-shift_y*sin(0-psi));    
            ptsy[i] = (shift_x*sin(0-psi)+shift_y*cos(0-psi));
          }

          // Polyfit takes 2 VectorXd's as arguments, so cast ptsx into Eigen::VectorXd
          double* points_x = &ptsx[0];
          double* points_y = &ptsy[0];
          Eigen::Map<Eigen::VectorXd> ptsx_vector(points_x, 6);
          Eigen::Map<Eigen::VectorXd> ptsy_vector(points_y, 6);

          // fit 3rd-order polynomial through the 6 waypoints -- defines line we want to follow
          auto coeffs = polyfit(ptsx_vector, ptsy_vector, 3);

          // CTE calc. Car is translated to (0,0), so eval coeffs at x=0.
          // Note: this isn't really a "cte", it's a y point that represents distance from trajectory,
          // since the car & trajectory were translated into horiztonal above
          double cte = polyeval(coeffs, 0);   
          
          // double epsi = psi - atan(coeffs[1] + 2*px*coeffs[2] + 3*coeffs[3]*pow(px,2));
          double epsi = -atan(coeffs[1]);   // since psi=0, and px=0 after translation

        
          
          // Current state (can use with latency calcs above)
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;   //x, y, psi, v, cte, epsi; x,y,psi=0 due to translation above


          // Solve optimal for (delta, a, MPS coords) based on (current state, waypoint coeffs)
          // Returns vector of <optimal delta, a, MPC predicted (x,y) coords>
          auto vars = mpc.Solve(state, coeffs);   

          
          /***** FOR VISUAL DEBUGGING ONLY *****/

          // Display the upcoming waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          double poly_inc = 2.5;    // x-value increment
          int num_points = 25;
          for (int i=1; i<num_points; i++) {
            next_x_vals.push_back(poly_inc*i);
            next_y_vals.push_back(polyeval(coeffs, poly_inc*i));

          }

          // Display the upcoming MPC-calculated waypoints (based on motion model predictions in MPC.cpp)
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          
          // Start i=2 b/c first vars are delta... see MPC::Solve for the vars vector that is returned
          // Every even value is x, every odd value is y
          for (int i=2; i < vars.size(); i++) {   
            if (i%2 == 0) {   //  even values, push an x
              mpc_x_vals.push_back(vars[i]);
            } else {          //  odd values, push a y
              mpc_y_vals.push_back(vars[i]);
            }
          
          }


          /***** END DEBUGGING *****/

          json msgJson;
          
          
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = vars[0]/(deg2rad(25)*Lf);   // See MPC.cpp for Lf
          msgJson["throttle"] = vars[1];
          
        

          /***** END MY CODE *****/

          
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;


          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          



          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does not actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.

          this_thread::sleep_for(chrono::milliseconds(100));
      

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
