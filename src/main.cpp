#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <tuple>
#include <string>
#include "Eigen/Core"
#include "Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;
using namespace std;

constexpr double mph2ISU = 1609.344 / 3600; // Factor for conversion from mph to meters/second

/**
 * @return the value of constant pi
 */
constexpr double pi() {
	return M_PI;
}

/**
 * Conversion from degrees to radians
 * @param x the angle to be converted, in degrees
 * @return the given angle expressed in radians
 */
double deg2rad(double x) {
	return x * pi() / 180;
}

/**
 * Conversion from radians to degrees
 * @param x the angle to be converted, in radians
 * @return the given angle expressed in degrees
 */
double rad2deg(double x) {
	return x * 180 / pi();
}

/**
 * Returns the number of milliseconds elapsed since the epoch.
 */
long long getCurrentTimestamp() {
	long long millisecondsSinceEpoch = std::chrono::duration_cast<
			std::chrono::milliseconds>(
			std::chrono::system_clock::now().time_since_epoch()).count();
	return millisecondsSinceEpoch;
}

/**
 * Changes the reference system of a 2D point, by translating and then rotating it.
 * @param x starting x coordinate of the given point
 * @param y starting y coordinate of the given point
 * @param x0 the x coordinate of the new origin, expressed in the starting reference system
 * @param y0 the y coordinate of the new origin, expressed in the starting reference system
 * @param psi0 rotation of the starting coordinates system in radians; positive is clockwise
 * @return a pair, holding respectively the x and y coordinates of the given point expressed in the new
 * reference system, after translation and rotation
 */
std::pair<double, double> getInNewRefSystem(const double x, const double y,
		const double x0, const double y0, const double psi0) {
	auto xShifted = x - x0;
	auto yShifted = y - y0;
	auto xRotated = xShifted * cos(-psi0) - yShifted * sin(-psi0);
	auto yRotated = xShifted * sin(-psi0) + yShifted * cos(-psi0);
	auto ret = std::make_pair(xRotated, yRotated);
	return ret;
}

/**
 * Given position and yaw (heading) of a car in a given reference system, determines its position and yaw in
 * a new reference system, obtained first translating and then rotating the given reference system.
 * @param x starting x coordinate of the car
 * @param y starting y coordinate of the car
 * @param psi starting yaw of the car expressed in radians, positive is counter-clockwise from the direction of the x axis
 * @param x0 the x coordinate of the new origin, expressed in the starting reference system
 * @param y0 the y coordinate of the new origin, expressed in the starting reference system
 * @param psi0 rotation of the starting coordinates system in radians; positive is clockwise
 * @return a tuple <x1, y1, psi1>, respectively the x and y coordinates and yaw of the car expressed in the new
 * reference system, after translation and rotation; note that psi1 could be negative
 */
std::tuple<double, double, double> getInNewRefSystem(const double x,
		const double y, const double psi, const double x0, const double y0,
		const double psi0) {
	auto transformed = getInNewRefSystem(x, y, x0, y0, psi0);
	auto xTransf = transformed.first;
	auto yTransf = transformed.second;
	auto psiTransf = psi - psi0;
	auto ret = std::make_tuple(xTransf, yTransf, psiTransf);
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

/**
 * Determines the value of a polynomial with given coefficients.
 * @param coeffs the polynomial coefficients, in order starting from the term of degree 0
 * @param x the value for which the polynomyal has to be evaluated
 * @return the given polynomial value in x
 */
double polyeval(Eigen::VectorXd coeffs, double x) {
	double result = 0.0;
	for (int i = 0; i < coeffs.size(); i++) {
		result += coeffs[i] * pow(x, i);
	}
	return result;
}

/**
 * Fits a polynomial to 2D points.
 * Adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
 *
 * @param xvals the x coordinates of the points to be fit
 * @param yvals the y coordinates of the points to be fit
 * @param order the order of the fitting polynomial
 * @return the fitting polynomial coefficients, in order starting from the term of degree 0
 */
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

void paramsErrorAndExit() {
	cout << "Usage: mpd [latency]" << endl
			<< "   [latency] an integer number giving the latency time in milliseconds. Defaults to 100."
			<< endl;
	exit(-1);
}

int main(int argc, char ** argv) {
	// Copy command line parameters into vector `args`, args[0] being the executable
	vector<string> args(argv, argv + argc);
	if (args.size() != 1 && args.size() != 2)
		paramsErrorAndExit();
	const int latency { (args.size() == 2) ? std::stoi(args[1]) : 100 }; // Latency in milliseconds
	if (latency < 0)
		paramsErrorAndExit();
	cout << "Running with a latency of " << latency << " milliseconds." << endl;

	uWS::Hub h;
	MPC mpc;
	constexpr double Lf = MPC::Lf;

	h.onMessage(
			[&mpc, &latency, &Lf](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
					uWS::OpCode opCode) {
				// "42" at the start of the message means there's a websocket message event.
				// The 4 signifies a websocket message
				// The 2 signifies a websocket event
				string sdata = string(data).substr(0, length);
				if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
					string s = hasData(sdata);
					if (s != "") {
						auto j = json::parse(s);
						string event = j[0].get<string>();
						if (event == "telemetry") {
							// j[1] is the data JSON object
							vector<double> ptsx = j[1]["ptsx"];
							vector<double> ptsy = j[1]["ptsy"];
							const double px = j[1]["x"];  // In meters
							const double py = j[1]["y"];  // In meters
							const double psi = j[1]["psi"];  // In radians
							const double v = j[1]["speed"];  // In miles per hour (mph)
							const double steeringAngle = j[1]["steering_angle"];  // In [-1, 1], corresponding to [-25deg, 25deg]

							// Convert steering angle from [-1, 1], as received from the simulator, to radians
							const double steeringAngleRad=deg2rad(25*steeringAngle);

							/*
							 * Convert the waypoints (received from the simulator) to the car reference system: origin on the
							 * car position and x axis along the camera yaw (heading). In the new reference system, the
							 * car poise has coordinates (0, 0, 0).
							 */
							for (auto i=0u; i<ptsx.size(); ++i) {
								auto transformed = getInNewRefSystem(ptsx[i], ptsy[i], px, py, psi);
								ptsx[i]=transformed.first;
								ptsy[i]=transformed.second;
							}

							/*
							 * Interpolate the waypoints (in the car reference system) with a cubic,
							 * and get the coefficients of the result.
							 */
							Eigen::Map<Eigen::VectorXd> xWaypoints(&ptsx[0], ptsx.size());
							Eigen::Map<Eigen::VectorXd> yWaypoints(&ptsy[0], ptsy.size());
							const auto coeffs = polyfit(xWaypoints, yWaypoints, 3);

							/*
							 * Calculate errors in car reference system
							 */
							const auto cte = polyeval(coeffs, 0); // Approximation, but good enough
							const auto epsy = -atan(coeffs[1]);

							/*
							 * Determine the car state as predicted at the end of the latency time interval.
							 * It is expressed in the reference system of the current car poise (i.e. at
							 * the beginning of the latency time interval); same reference system waypoints
							 * have been converted to).
							 */
							const double vISU=v*mph2ISU;  // Convert speed from mph to meters/second (International System of Units)
							const auto pxPred = vISU*latency/1000;  // To keep the position in meters, need to use the velocity in meters/second
							const auto pyPred = 0;
							const auto psiPred = -(v/Lf)*steeringAngleRad*latency/1000.;  // Assuming constant steering angle
							const auto vPred = v; // Assuming no acceleration, for simplification
							const auto ctePred = cte+v*sin(epsy)*latency/1000.;
							const auto epsyPred = epsy+psiPred;

							Eigen::VectorXd state {6};  // The vector to hold the predicted car state
							state << pxPred, pyPred, psiPred, vPred, ctePred, epsyPred;

							// Run the optimisation given the state vector and polynomial interpolation coefficients
							auto optResult = mpc.Solve(state, coeffs);  // Note that state[3] is in mph

							/*
							 * Fill in information for the simulator to draw the polynomial interpolating
							 * the waypoints (in yellow).
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

							/*
							 * Fill in the MPC output, to be sent to the simulator for visualisation (in green). Include
							 * the predicted car position as the first point.
							 */
							// Predicted and optimised trajectory
							vector<double> x {pxPred};
							vector<double> y {pyPred};
							for (auto i=2u; i<optResult.size(); ++i) {
								if (i%2==0) x.push_back(optResult[i]);
								else y.push_back(optResult[i]);
							}

							// Steering and throttle values, both to be in [-1, 1] for the simulator
							const double steerValue=optResult[0]/(deg2rad(25)*Lf);
							const double throttleValue= optResult[1];

							json msgJson;
							msgJson["steering_angle"] = steerValue;
							msgJson["throttle"] = throttleValue;

							/*
							 * The simulator expects points in the vehicle's coordinates system
							 */
							msgJson["mpc_x"] = x;
							msgJson["mpc_y"] = y;
							msgJson["next_x"] = xTrack;
							msgJson["next_y"] = yTrack;

							auto msg = "42[\"steer\"," + msgJson.dump() + "]";
							// Latency
							// The purpose is to mimic real driving conditions where
							// the car does not actuate the commands instantly.
							//
							// Feel free to play around with this value but should be to drive
							// around the track with 100ms latency.
							this_thread::sleep_for(chrono::milliseconds(latency));
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
