#include <eigen3/Eigen/Dense>
#include <iostream>

using namespace std;

class kalman_filter
{
private:
    //state vector
    Eigen::VectorXd x_;
    //state transistion matrix
    Eigen::MatrixXd F_;
    //state covariance matrix
    Eigen::MatrixXd P_;
    //process covariance matrix
    Eigen::MatrixXd Q_;
    //measurement matrix
    Eigen::MatrixXd H_;
    //measurement covariance matrix
    Eigen::MatrixXd R_;

public:
    //constructor
    kalman_filter();
    //destructor
    //~kalman_filter();

    //initialize state vector
    void Initialization(Eigen::VectorXd x_in) {
        x_ = x_in;
    }
    //set state transistion matrix
    void SetF(Eigen::MatrixXd F_in) {
        F_ = F_in;
    }
    //set state covariance matrix
    void SetP(Eigen::MatrixXd P_in) {
        P_ = P_in;
    }
    //set process covariance matrix
    void SetQ(Eigen::MatrixXd Q_in) {
        Q_ = Q_in;
    }
    //set measurement matrix
    void SetH(Eigen::MatrixXd H_in) {
        H_ = H_in;
    }
    //measurement covariance matrix
    void SetR(Eigen::MatrixXd R_in) {
        R_ = R_in;
    }
    //return vehicle state
    Eigen::VectorXd GetX() {
        return x_;
        cout << x_ << endl;
    }
    //define prediction function
    void Prediction();
    void Update(const Eigen::VectorXd& z);
};

