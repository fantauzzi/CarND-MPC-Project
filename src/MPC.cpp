// Code below reworked from Udacity quizzes about MPC, in their Self-Driving Car program.

#include "MPC.h"
#include <limits>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen/Core"

using CppAD::AD;

// Necessary definitions for static data members
constexpr double MPC::dt;
constexpr double MPC::refCte;
constexpr double MPC::refEpsi;
constexpr double MPC::refV;
constexpr double MPC::Lf;

MPC::FG_eval::FG_eval(Eigen::VectorXd coeffs) {
	this->coeffs = coeffs;
}

void MPC::FG_eval::operator()(ADvector& fg, const ADvector& vars) {
	// `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
	fg[0] = 0;

	// The part of the cost based on the reference state.
	for (auto i = 0u; i < N; i++) {
		fg[0] += 2000 * CppAD::pow(vars[cteStart + i] - refCte, 2);
		fg[0] += 2000 * CppAD::pow(vars[epsiStart + i] - refEpsi, 2);
		fg[0] += 1.2 * CppAD::pow(vars[vStart + i] - refV, 2);
	}

	// Minimize the use of actuators.
	for (auto i = 0u; i < N - 1; i++) {
		fg[0] += 5 * CppAD::pow(vars[deltaStart + i], 2);
		fg[0] += 5 * CppAD::pow(vars[aStart + i], 2);
	}

	// Minimize the value gap between sequential actuations.
	for (auto i = 0u; i < N - 2; i++) {
		fg[0] += 200
				* CppAD::pow(vars[deltaStart + i + 1] - vars[deltaStart + i],
						2);
		fg[0] += 10 * CppAD::pow(vars[aStart + i + 1] - vars[aStart + i], 2);
	}

	// Initial constraints
	//
	// We add 1 to each of the starting indices due to cost being located at
	// index 0 of `fg`.
	// This bumps up the position of all the other values.
	fg[1 + xStart] = vars[xStart];
	fg[1 + yStart] = vars[yStart];
	fg[1 + psiStart] = vars[psiStart];
	fg[1 + vStart] = vars[vStart];
	fg[1 + cteStart] = vars[cteStart];
	fg[1 + epsiStart] = vars[epsiStart];

	for (auto i = 1u; i < N; i++) {
		// The state at time t+1 .
		AD<double> x1 = vars[xStart + i];
		AD<double> y1 = vars[yStart + i];
		AD<double> psi1 = vars[psiStart + i];
		AD<double> v1 = vars[vStart + i];
		AD<double> cte1 = vars[cteStart + i];
		AD<double> epsi1 = vars[epsiStart + i];

		// The state at time t.
		AD<double> x0 = vars[xStart + i - 1];
		AD<double> y0 = vars[yStart + i - 1];
		AD<double> psi0 = vars[psiStart + i - 1];
		AD<double> v0 = vars[vStart + i - 1];
		AD<double> cte0 = vars[cteStart + i - 1];
		AD<double> epsi0 = vars[epsiStart + i - 1];

		// Only consider the actuation at time t.
		AD<double> delta0 = vars[deltaStart + i - 1];
		AD<double> a0 = vars[aStart + i - 1];

		AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0
				+ coeffs[3] * x0 * x0 * x0;
		AD<double> psides0 = CppAD::atan(
				3 * coeffs[3] * x0 * x0 + 2 * coeffs[2] * x0 + coeffs[1]);

		// Recall the equations for the model:
		// x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
		// y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
		// psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
		// v_[t+1] = v[t] + a[t] * dt
		// cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
		// epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
		fg[1 + xStart + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
		fg[1 + yStart + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
		fg[1 + psiStart + i] = psi1 - (psi0 - v0 * delta0 / Lf * dt);
		fg[1 + vStart + i] = v1 - (v0 + a0 * dt);
		fg[1 + cteStart + i] = cte1
				- ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
		fg[1 + epsiStart + i] = epsi1
				- ((psi0 - psides0) - v0 * delta0 / Lf * dt);
	}
}

std::vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
	typedef CPPAD_TESTVECTOR(double)Dvector;

	double x = state[0];
	double y = state[1];
	double psi = state[2];
	double v = state[3];
	double cte = state[4];
	double epsi = state[5];

	size_t n_vars = N*6+(N-1)*2;  // Number of optimisation variables

	size_t n_constraints = N*6;// Number of constraints

	// Initial value of the independent variables.
	// SHOULD BE 0 besides initial state.
	Dvector vars(n_vars);
	for (auto i = 0u; i < n_vars; i++) {
		vars[i] = 0;
	}

	// Set lower and upper limits for variables.
	Dvector vars_lowerbound(n_vars);
	Dvector vars_upperbound(n_vars);

	// Set all non-actuators upper and lowerlimits
	// to the max negative and positive values.
	for (auto i = 0u; i < deltaStart; i++) {
		vars_lowerbound[i] = std::numeric_limits<double>::lowest();
		vars_upperbound[i] = std::numeric_limits<double>::max();
	}

	// The upper and lower limits of delta are set to -25 and 25
	// degrees (values in radians).
	for (auto i = deltaStart; i < aStart; i++) {
		vars_lowerbound[i] = -0.436332*Lf;
		vars_upperbound[i] = 0.436332*Lf;
	}

	// Throttle/break upper and lower limits.
	for (auto i = aStart; i < n_vars; i++) {
		vars_lowerbound[i] = -1.0;
		vars_upperbound[i] = 1.0;
	}

	// Lower and upper limits for the constraints
	// Should be 0 besides initial state.
	Dvector constraints_lowerbound(n_constraints);
	Dvector constraints_upperbound(n_constraints);
	for (auto i = 0u; i < n_constraints; i++) {
		constraints_lowerbound[i] = 0;
		constraints_upperbound[i] = 0;
	}

	constraints_lowerbound[xStart] = x;
	constraints_lowerbound[yStart] = y;
	constraints_lowerbound[psiStart] = psi;
	constraints_lowerbound[vStart] = v;
	constraints_lowerbound[cteStart] = cte;
	constraints_lowerbound[epsiStart] = epsi;

	constraints_upperbound[xStart] = x;
	constraints_upperbound[yStart] = y;
	constraints_upperbound[psiStart] = psi;
	constraints_upperbound[vStart] = v;
	constraints_upperbound[cteStart] = cte;
	constraints_upperbound[epsiStart] = epsi;

	// Object that computes objective and constraints
	FG_eval fg_eval(coeffs);

	// options for IPOPT solver
	std::string options;
	// Change this if you'd like more print information
	options += "Integer print_level  0\n";
	// NOTE: Setting sparse to true allows the solver to take advantage
	// of sparse routines, this makes the computation MUCH FASTER. If you
	// can uncomment 1 of these and see if it makes a difference or not but
	// if you uncomment both the computation time should go up in orders of
	// magnitude.
	options += "Sparse  true        forward\n";
	options += "Sparse  true        reverse\n";
	// Time limit before the solver gives up and returns the best solution found so far
	options += "Numeric max_cpu_time          0.5\n";// In seconds

	// Will hold the solution
	CppAD::ipopt::solve_result<Dvector> solution;

	// Solve the optimisation problem
	CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound,
			vars_upperbound, constraints_lowerbound, constraints_upperbound,
			fg_eval, solution);

	// Return the first actuator values.

	std::vector<double> result;
	result.push_back(solution.x[deltaStart]);
	result.push_back(solution.x[aStart]);

	for (auto i=0u; i<N-1; ++i) {
		result.push_back(solution.x[xStart+i+1]);
		result.push_back(solution.x[yStart+i+1]);
	}
	return result;
}
