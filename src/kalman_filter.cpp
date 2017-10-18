#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
    
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    
    VectorXd y = z - H_* x_;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_ * Ht * Si;
    //new state
    x_ = x_ + (K * y);
    MatrixXd I = MatrixXd::Identity(4, 4);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);
    
    float c1 = sqrt(px*px+py*py);
    float c2 = atan2(py, px);
    if (c1 < 0.0000001)
    {
        return;
    }
    float c3 = (px*vx+py*vy)/c1;
    
    VectorXd h(3);
    h << c1, c2, c3;
    
    VectorXd y = z - h;
    // shifting y[1] to [-pi, pi] range
    float PI = 2*acos(0.0);
    float phi = y[1] + PI;
    int dev = (int) phi/(2*PI);
    float phi_ = phi - 2*PI*((float) dev);
    phi = phi_ - PI;
    y[1] = phi;
    //
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_ * Ht * Si;
    
    //new state
    x_ = x_ + (K * y);
    MatrixXd I = MatrixXd::Identity(4, 4);
    P_ = (I - K * H_) * P_;
}
