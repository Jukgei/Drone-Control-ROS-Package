#ifndef MPC_H
#define MPC_H
#include <iostream>
#include <vector>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>

using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

namespace FlightControl{

class mpc
{
public:
    mpc();
    std::vector<float> mpcController();
    ~mpc() {}

private:


    void GaussNewtonMutipleShooting();




    int Horizon;
    int controlDim;
    int stateDim;
    float simulationDeltaT;
    int simulationPoint;
    int shootingInterval;
    bool Identification;
    
    Eigen::VectorXf xInit;
    Eigen::VectorXf uInit;

    
    Eigen::VectorXf xRef;
    Eigen::VectorXf uRef;
    
    Eigen::VectorXf qVector; 
    Eigen::VectorXf qVector_; 
    Eigen::VectorXf rVector;
    
    Eigen::MatrixXf Q;
    Eigen::MatrixXf Q_;
    Eigen::MatrixXf R;

};


}

#endif
