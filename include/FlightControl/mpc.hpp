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
    std::vector<float> mpcController(Eigen::VectorXf x, Eigen::VectorXf uPast);
    ~mpc() {}


private:


    void GaussNewtonMutipleShooting();
    void RolloutShot(int startIndex, int endIndex,
                     Eigen::MatrixXf &xTraj,
                     Eigen::MatrixXf &uTraj,
                     Eigen::MatrixXf &xReference,
                     Eigen::MatrixXf &stateOverWrite,
                     Eigen::MatrixXf &inputOverWrite,
                     std::vector<Eigen::MatrixXf> &L);

    void RolloutSingleShot(int n,
                           Eigen::MatrixXf &xTraj,
                           Eigen::MatrixXf &uTraj,
                           Eigen::MatrixXf &xReference,
                           Eigen::MatrixXf &stateOverWrite,
                           Eigen::MatrixXf &inputOverWrite,
                           std::vector<Eigen::MatrixXf> &L);
    Eigen::VectorXf ComputeSingleDefect();
    void UpdateCost();
    void LQApproximate();
    void BackwardIteration();
    void ComputeStateAndControl();
    Eigen::MatrixXf RK45(Eigen::MatrixXf (*f)(Eigen::VectorXf x,Eigen::VectorXf u),
                         float tStart,
                         float tEnd,
                         Eigen::VectorXf x,
                         Eigen::VectorXf u);

    Eigen::MatrixXf Dynamic(Eigen::VectorXf x,Eigen::VectorXf u);


    int Horizon;
    int controlDim;
    int stateDim;
    float simulationDeltaT;
    int simulationPoint;
    int shootingInterval;
    bool Identification;
    
    Eigen::VectorXf xInit;
    Eigen::VectorXf uInit;

    Eigen::MatrixXf xInitTraj;
    Eigen::MatrixXf uInitTraj;
   
    Eigen::VectorXf xRef;
    Eigen::VectorXf uRef;
    
    Eigen::VectorXf xRefTraj;
    Eigen::MatrixXf uRefTraj;
    
    Eigen::VectorXf qVector; 
    Eigen::VectorXf qVector_; 
    Eigen::VectorXf rVector;
    
    Eigen::MatrixXf Q;
    Eigen::MatrixXf Q_;
    Eigen::MatrixXf R;

    std::vector<Eigen::MatrixXf> xOverWrite;
    std::vector<Eigen::MatrixXf> uOverWrite;

    Eigen::MatrixXf d;
    
    float m;
    float g_;
};


}

#endif
