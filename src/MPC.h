#pragma once
#include <vector>
#include "Eigen/Core"

using namespace std;

class MPC {
public:

	MPC();

	virtual ~MPC();

	virtual double getLf() const;

	// Solve the model given an initial state and polynomial coefficients.
	// Return the first actuatotions.
	vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};
