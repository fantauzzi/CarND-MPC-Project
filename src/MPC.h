#pragma once
#include <vector>
#include "Eigen/Core"
#include <cppad/cppad.hpp>

using CppAD::AD;

class MPC {
	static constexpr size_t N = 11; // Number of time steps to be used in the optimisation
	static constexpr double dt = .13; // Number of time steps to be used in the optimisation

	static constexpr double refCte { 0 };
	static constexpr double refEpsi { 0 };
	static constexpr double refV { 100 };  // Target speed in mph

	static constexpr size_t xStart { 0 };
	static constexpr size_t yStart { xStart + N };
	static constexpr size_t psiStart { yStart + N };
	static constexpr size_t vStart { psiStart + N };
	static constexpr size_t cteStart { vStart + N };
	static constexpr size_t epsiStart { cteStart + N };
	static constexpr size_t deltaStart { epsiStart + N };
	static constexpr size_t aStart { deltaStart + N - 1 };

	class FG_eval {
		Eigen::VectorXd coeffs;
	public:
		FG_eval(Eigen::VectorXd coeffs);
		typedef CPPAD_TESTVECTOR(AD<double>)ADvector;
		void operator()(ADvector& fg, const ADvector& vars);
	};

public:
	// This value assumes the model presented in the classroom is used.
	//
	// It was obtained by measuring the radius formed by running the vehicle in the
	// simulator around in a circle with a constant steering angle and velocity on a
	// flat terrain.
	//
	// Lf was tuned until the the radius formed by the simulating the model
	// presented in the classroom matched the previous radius.
	//
	// This is the length from front to CoG that has a similar radius.
	static constexpr double Lf {2.67};

	// Solve the model given an initial state and polynomial coefficients.
	// Return the first actuatotions.
	std::vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};
