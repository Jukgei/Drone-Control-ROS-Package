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
    mpc(int m,int n, int horizon,float mass, float g,
        Eigen::VectorXf vQ,
        Eigen::VectorXf vQ_,
        Eigen::VectorXf vR);
    std::vector<float> mpcController(Eigen::VectorXf x, Eigen::VectorXf uPast,
                                     Eigen::VectorXf xRef);
    ~mpc() {}


private:


    void GaussNewtonMutipleShooting();
    void RolloutShot(int startIndex, int endIndex,
                     const Eigen::MatrixXf &xTraj,
                     const Eigen::MatrixXf &uTraj,
                     const Eigen::MatrixXf &xReference,
                     Eigen::MatrixXf &stateOverWrite,
                     Eigen::MatrixXf &inputOverWrite,
                     std::vector<Eigen::MatrixXf> &L);

    void RolloutSingleShot(int n,
                           const Eigen::MatrixXf &xTraj,
                           const Eigen::MatrixXf &uTraj,
                           const Eigen::MatrixXf &xReference,
                           Eigen::MatrixXf &stateOverWrite,
                           Eigen::MatrixXf &inputOverWrite,
                           std::vector<Eigen::MatrixXf> &L);

    Eigen::VectorXf ComputeSingleDefect(int n,
                                        const Eigen::MatrixXf &stateOverWrite,
                                        const Eigen::MatrixXf &xTraj);
    
    void UpdateCost();
    void LQApproximate(const Eigen::MatrixXf &xTraj,
                       const Eigen::MatrixXf &uTraj);

    void BackwardIteration();
    void ComputeStateAndControl(Eigen::MatrixXf &diffu,
                                Eigen::MatrixXf &diffx);

    Eigen::MatrixXf RK45(Eigen::VectorXf (mpc::*f)(const Eigen::VectorXf &x,const Eigen::VectorXf &u),
                         float tStart,
                         float tEnd,
                         const Eigen::VectorXf &x,
                         const Eigen::VectorXf &u);

    Eigen::VectorXf Dynamic(const Eigen::VectorXf &x, const Eigen::VectorXf &u);
    
    void LinearSystem(const Eigen::VectorXf xTraj,
                      const Eigen::VectorXf uTraj,
                      Eigen::MatrixXf &A,
                      Eigen::MatrixXf &B);


    Eigen::VectorXf (mpc::*pDynamic)(const Eigen::VectorXf &x, const Eigen::VectorXf &u);

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
   
    //Eigen::VectorXf xRef;
    //Eigen::VectorXf uRef;
    
    Eigen::VectorXf xRefTraj;
    Eigen::MatrixXf uRefTraj;
    
    Eigen::VectorXf qVector; 
    Eigen::VectorXf qVector_; 
    Eigen::VectorXf rVector;
    
    Eigen::MatrixXf Qweight;
    Eigen::MatrixXf Q_;
    Eigen::MatrixXf Rweight;

    std::vector<Eigen::MatrixXf> xOverWrite;
    std::vector<Eigen::MatrixXf> uOverWrite;

    std::vector<Eigen::MatrixXf> A;
    std::vector<Eigen::MatrixXf> B;
    std::vector<Eigen::MatrixXf> Q;
    std::vector<Eigen::MatrixXf> R;
    std::vector<Eigen::MatrixXf> P;
    std::vector<Eigen::MatrixXf> q;
    std::vector<Eigen::MatrixXf> r;
   
    std::vector<Eigen::MatrixXf> S;
    std::vector<Eigen::MatrixXf> s;
    std::vector<Eigen::MatrixXf> h;
    std::vector<Eigen::MatrixXf> G;
    std::vector<Eigen::MatrixXf> H;
    std::vector<Eigen::MatrixXf> l;
    std::vector<Eigen::MatrixXf> L;
    

    Eigen::MatrixXf d;
    
    float m;
    float g_;
};

}

#endif
