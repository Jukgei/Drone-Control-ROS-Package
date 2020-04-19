#include "../include/FlightControl/mpc.hpp"

FlightControl::mpc::mpc(){
    
}

std::vector<float> FlightControl::mpc::mpcController(){
    //set xRef and uRef 
    this->uRef = Eigen::MatrixXf::Zero(controlDim,Horizon);   
    this->xRef = Eigen::MatrixXf::Ones(stateDim,Horizon);   

    //set xInit and uInit
    
    //GNMS
    GaussNewtonMutipleShooting();

    std::vector<float> tmp ;
    return tmp;
}

void FlightControl::mpc::GaussNewtonMutipleShooting(){
    //Initialize guess point use a trival controller
    
    
    //Initialize x and u result
    
    
    
    //rolloutshot
    
    
    
    //update cost
    
    
    
    
    //compute defect
    
    
    
    //LQ problem approximation
    
    
    //Backforward Iteration: find S_N S_n l_n and L_N 
    
    
    
    //update delta x and delta u
    
    
    
    
    //line Search:
        // 1.compute cost
        // 2.update the full control and state
        // 3.reset the defect
        // 4.rolloutshot
        // 5.update cost
        // 6.compute the norm of the  delta u and the delta x
        // 7.compute J diff
    
    
    
}

