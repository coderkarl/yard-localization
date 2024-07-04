#include <Eigen/Dense>

#pragma once

namespace yard {

class EKFPose
{
public:
	EKFPose(size_t num_states, size_t num_meas, size_t num_inputs);
	~EKFPose();

	void init(const Eigen::VectorXd& initX,
	        const Eigen::MatrixXd& initP, 
			const Eigen::MatrixXd& inputNoiseQ,
			const Eigen::MatrixXd& measNoiseR);

	// Use tf to calculate new_state = map to base_link
	//   Set internal state = new_state and upate F, G, H
	void update(const Eigen::VectorXd& new_state, double distance, double dt);

	void updateExpectedMeasure(double landmark_x, double landmark_y);

	// Measurement is [landmark xy in base_link frame]
	void applyMeasure(double mx, double my);

	Eigen::VectorXd state() { return x_state; };
	Eigen::MatrixXd getP() { return P; };
	double x() { return x_state(0); };
	double y() { return x_state(1); };
	double yaw_rad() { return x_state(2); };

private:
  Eigen::VectorXd x_state;

  Eigen::MatrixXd P; // state covariance
  Eigen::MatrixXd Q; // input noise
  Eigen::MatrixXd R; // measurement noise

  Eigen::MatrixXd F; // Jacobian state w.r.t. state
  Eigen::MatrixXd G; // Jacobian state w.r.t. input
  Eigen::MatrixXd H; // Jacobian measurement w.r.t. state
  Eigen::MatrixXd K; // Kalman gain

  Eigen::VectorXd expected_meas;

};

} // namespace yard
