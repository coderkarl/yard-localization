// Copyright 2019 coderkarl. Subject to the BSD license.

#include "yard_slam/EKFPose.h"

namespace yard {

EKFPose::EKFPose(size_t num_states, size_t num_meas, size_t num_inputs)
{
  x_state.resize(num_states);
  P.resize(num_states, num_states);
  Q.resize(num_inputs, num_inputs);
  R.resize(num_meas, num_meas);
}

EKFPose::~EKFPose() {}

void EKFPose::init(const Eigen::VectorXd& initX,
		const Eigen::MatrixXd& initP, 
		const Eigen::MatrixXd& inputNoiseQ,
		const Eigen::MatrixXd& measNoiseR)
{
  x_state = initX;
  P = initP;
  Q = inputNoiseQ;
  R = measNoiseR;

  size_t n = P.rows();
  size_t m = R.rows();
  F.resize(n,n);
  G.resize(n,m);
  H.resize(m,n);
  K.resize(n,m);

  expected_meas.resize(m);
}

void EKFPose::update(const Eigen::VectorXd& new_state, double distance, double dt) {
  x_state = new_state;
  
  double cY = cos(yaw_rad()); //cache in member variable ?
  double sY = sin(yaw_rad());
  F << 1, 0, -distance * sY,
       0, 1,  distance * cY,
	   0, 0,  1;

  G << cY, 0,
       sY, 0,
	    0, dt;
  
  P = F*P*F.transpose() + G*Q*G.transpose();
}

void EKFPose::updateExpectedMeasure(double landmark_x, double landmark_y) {
  double cY = cos(yaw_rad()); //cache in member variable ?
  double sY = sin(yaw_rad());
  double dX = landmark_x - x();
  double dY = landmark_y - y();
  H << -cY, -sY, -dX * sY + dY * cY,
        sY, -cY,  dX * cY - dY * sY;

  expected_meas << cY * dX + sY * dY,
                  -sY * dX + cY * dY;
}

void EKFPose::applyMeasure(double mx, double my) {
  Eigen::MatrixXd prevP = P;
  Eigen::VectorXd prev_state = x_state;

  Eigen::MatrixXd S = H*P*H.transpose() + R;
  K = P*H.transpose()*(S.inverse());
  P = P - K*S*K.transpose();

  Eigen::VectorXd Z(2);
  Z << mx, my;
  x_state = x_state + K*(Z - expected_meas);
  if (std::abs(x_state(0) - prev_state(0)) > 1.0) {
    x_state = prev_state;
    P = prevP;
  } else if (std::abs(x_state(1) - prev_state(1)) > 1.0) {
    x_state = prev_state;
    P = prevP;
  } else {
    double yaw_diff = x_state(2) - prev_state(2);
    if (yaw_diff > M_PI) {yaw_diff -= 2*M_PI;}
    if (yaw_diff < -M_PI) {yaw_diff += 2*M_PI;}
    if (std::abs(yaw_diff) > 5.0) {
      x_state = prev_state;
      P = prevP;
    }
  }

}

} // namespace yard
