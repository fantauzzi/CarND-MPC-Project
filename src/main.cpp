#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <tuple>
#include "Eigen/Core"
#include "Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() {
	return M_PI;
}
double deg2rad(double x) {
	return x * pi() / 180;
}
double rad2deg(double x) {
	return x * 180 / pi();
}

long long getCurrentTimestamp() {
	long long millisecondsSinceEpoch = std::chrono::duration_cast<
			std::chrono::milliseconds>(
			std::chrono::system_clock::now().time_since_epoch()).count();
	return millisecondsSinceEpoch;
}

std::pair<double, double> getInNewRefSystem(const double x, const double y, const double x0, const double y0, const double psi0) {
	auto xShifted=x-x0;
	auto yShifted=y-y0;
	auto xRotated=xShifted*cos(-psi0)-yShifted*sin(-psi0);
	auto yRotated=xShifted*sin(-psi0)+yShifted*cos(-psi0);
	auto ret=std::make_pair(xRotated, yRotated);
	return ret;
}

std::tuple<double, double, double> getInNewRefSystem(const double x, const double y, const double psi,const double x0, const double y0, const double psi0) {
	auto transformed = getInNewRefSystem(x, y, x0, y0, psi0);
	auto xTransf=transformed.first;
	auto yTransf=transformed.second;
	auto psiTransf= psi-psi0;
	//if (psiTransf<0) psiTransf+=2*pi();
	//assert(psiTransf>=0 && psiTransf<=2*pi());
	auto ret= std::make_tuple(xTransf, yTransf, psiTransf);
	return ret;
}


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

	long long timeStamp { getCurrentTimestamp() };
	double vPrevious { 0. };

	h.onMessage(
			[&mpc, &timeStamp, &vPrevious](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
					uWS::OpCode opCode) {
				// "42" at the start of the message means there's a websocket message event.
				// The 4 signifies a websocket message
				// The 2 signifies a websocket event
				string sdata = string(data).substr(0, length);
				// cout << sdata << endl;
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

							// Convert speed from mph to meters/second
							double vIS=v*1609.344/3600;

							double deltaT= (getCurrentTimestamp()- timeStamp)/1000.;
							timeStamp=getCurrentTimestamp();
							// double accel=(vIS-vPrevious)/deltaT;
							double accel=.0;
							vPrevious=vIS;

							// DONE use the two variables below to handle delay
							double steeringAngle = j[1]["steering_angle"];
							steeringAngle=deg2rad(25*steeringAngle);
							// cout << "Steering=" << steeringAngle << endl;
							// cout << "mph=" << v << "m/s=" << vIS << endl;
							// double throttle = j[1]["throttle"];
							constexpr double Lf=2.67;
							constexpr unsigned latency {100}; // Latency in milliseconds

							auto pxPred = px+(vIS*latency/1000.+.5*accel*pow(latency/1000.,2)) * cos(psi);
							auto pyPred = py+(vIS*latency/1000.+.5*accel*pow(latency/1000.,2)) * sin(psi);
							auto psiPred=psi + (v/Lf)*steeringAngle*latency/1000.;
							// cout << "Psi=" << rad2deg(psi) << " psiPred=" << rad2deg(psiPred) << endl;
							//auto vPred=vIS+accel*latency/1000.;

							/*
							 * Convert waypoints to the car reference system; i.e. car in (0,0,0), with x
							 * axis oriented as the car heading.
							 */

							for (auto i=0u; i<ptsx.size(); ++i) {
								/*
								// Translate first
								ptsx[i]-=pxPred;
								ptsy[i]-=pyPred;
								// Then rotate
								auto xRotated = ptsx[i]*cos(-psiPred)-ptsy[i]*sin(-psiPred);
								auto yRotated = ptsx[i]*sin(-psiPred)+ptsy[i]*cos(-psiPred);
								ptsx[i]=xRotated;
								ptsy[i]=yRotated;*/

								auto transformed = getInNewRefSystem(ptsx[i], ptsy[i], pxPred, pyPred, psiPred);
								ptsx[i]=transformed.first;
								ptsy[i]=transformed.second;
							}

							// Interpolate the waypoints with a polynomial and get the coefficients of the result
							Eigen::Map<Eigen::VectorXd> xWaypoints(&ptsx[0], ptsx.size());
							Eigen::Map<Eigen::VectorXd> yWaypoints(&ptsy[0], ptsy.size());
							auto coeffs = polyfit(xWaypoints, yWaypoints, 3);

							// Determine errors and state vector
							auto cte = polyeval(coeffs, 0);// Approximation, but good enough
							auto epsy = -atan(coeffs[1]);
							// auto epsy = psiPred-atan(coeffs[1]+2*pxPred*coeffs[2]+3*coeffs[3]*pow(pxPred,2));

							auto transfPoise = getInNewRefSystem(px, py, psi, pxPred, pyPred, psiPred);
							auto xTransf = std::get<0>(transfPoise);
							auto yTransf = std::get<1>(transfPoise);
							auto psiTransf = std::get<2>(transfPoise);
							Eigen::VectorXd state {6};
							// state << xTransf, yTransf, psiTransf, v, cte, epsy; // This assumes accel==0
							state << xTransf, yTransf, psiTransf, v, cte, epsy;  // This assumes accel==0

							// Run the optimisation given the state vector and polynomial interpolation coefficients
							auto optResult = mpc.Solve(state, coeffs);

							/*
							 * Fill in information to allow the simulator to draw the polynomial interpolating
							 * the waypoints.
							 */
							vector<double> xTrack;
							vector<double> yTrack;
							double trackSpacing=2.5;
							int numTrackPoints=25;
							for (auto i=1; i< numTrackPoints; ++i) {
								auto x = i*trackSpacing;
								xTrack.push_back(x);
								yTrack.push_back(polyeval(coeffs, x));
							}

							// Fill in computation output to be sent to the simulator
							vector<double> x;
							vector<double> y;
							x.push_back(xTransf);
							y.push_back(yTransf);

							for (auto i=2u; i<optResult.size(); ++i) {
								if (i%2==0) x.push_back(optResult[i]);
								else y.push_back(optResult[i]);
							}

							/*
							 * DONE: Calculate steering angle and throttle using MPC.
							 *
							 * Both are in between [-1, 1].
							 *
							 */
							double steer_value=optResult[0]/(deg2rad(25)*Lf); // TODO check Lf here!!
							assert(steer_value>=-1 && steer_value<=1);
							double throttle_value= optResult[1];
							assert(throttle_value>=-1 && throttle_value<=1);

							json msgJson;
							// NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
							// Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
							msgJson["steering_angle"] = steer_value;
							msgJson["throttle"] = throttle_value;

							//Display the MPC predicted trajectory
							//vector<double> mpc_x_vals;
							//vector<double> mpc_y_vals;

							//.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
							// the points in the simulator are connected by a Green line

							msgJson["mpc_x"] = x;
							msgJson["mpc_y"] = y;

							//Display the waypoints/reference line
							// vector<double> next_x_vals;
							// vector<double> next_y_vals;

							//.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
							// the points in the simulator are connected by a Yellow line

							msgJson["next_x"] = xTrack;
							msgJson["next_y"] = yTrack;

							auto msg = "42[\"steer\"," + msgJson.dump() + "]";
							// std::cout << msg << std::endl;
							// Latency
							// The purpose is to mimic real driving conditions where
							// the car does actuate the commands instantly.
							//
							// Feel free to play around with this value but should be to drive
							// around the track with 100ms latency.
							//
							// NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
							// SUBMITTING.
							this_thread::sleep_for(chrono::milliseconds(latency));
							ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
							cout << "====== DeltaT=" << deltaT << endl;
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
