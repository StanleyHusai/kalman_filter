#include "kalman_filter.h"
#include <iostream>
#include <eigen3/Eigen/Dense>

kalman_filter::kalman_filter()
{
    
}

void kalman_filter::Prediction() 
{
    //x'=Fx+u
    x_ = F_ * x_;
    //transpose matrix of state transistion matrix
    Eigen::MatrixXd Ft = F_.transpose();
    //P'=FPFt+Q
    P_ = F_ * P_ * Ft + Q_;
}

void kalman_filter::Update(const Eigen::VectorXd& z) 
{
    //y=z-Hx'
    Eigen::MatrixXd y = z - H_ * x_;
    //S=HP'Ht+R
    Eigen::MatrixXd S_ = H_ * P_ * H_.transpose() + R_;
    //K=P'HtSi
    Eigen::MatrixXd K = P_ * H_.transpose() * S_.inverse();
    //x=x'+Ky
    x_ = x_ + K * y;
    //P=(I-KH)P'
    int size = x_.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(size, size);
    P_ = (I - K * H_) * P_;
}