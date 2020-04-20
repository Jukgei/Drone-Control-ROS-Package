#include "../include/FlightControl/mpc.hpp"

FlightControl::mpc::mpc(){
    
}

std::vector<float> FlightControl::mpc::mpcController(Eigen::VectorXf x, Eigen::VectorXf uPast){
    //set xRef and uRef 
    this->uRefTraj = Eigen::MatrixXf::Zero(controlDim,Horizon-1);   
    this->xRefTraj = Eigen::MatrixXf::Zero(stateDim,Horizon);   
    for(int i = 0; i < Horizon -1 ;i ++){
        xRefTraj.block(0,i,stateDim,1) = xRef;
    }

    //set xInit and uInit
    this->xInitTraj = Eigen::MatrixXf::Zero(stateDim,Horizon);
    this->uInitTraj = Eigen::MatrixXf::Zero(controlDim,Horizon-1);
    for(int i = 0; i < Horizon ; i ++)
        xInitTraj.block(0,i,stateDim,1) = x;
    for(int i = 0; i < Horizon-1; i++)
        uInitTraj.block(0,i,controlDim,i) = uPast;

    //GNMS
    GaussNewtonMutipleShooting();

    std::vector<float> tmp ;
    return tmp;
}

void FlightControl::mpc::GaussNewtonMutipleShooting(){
    
    int m = controlDim;
    int n = stateDim;

    //Initialize guess point use a trival controller
    std::vector<Eigen::MatrixXf> L(Horizon);
    for(int i = 0; i < Horizon; i++)
        L[i] = Eigen::MatrixXf::Zero(m,n);
    
    //Initialize x and u result
    std::vector<Eigen::MatrixXf> xOverWriteRecord(2);
    std::vector<Eigen::MatrixXf> uOverWriteRecord(2);
    xOverWriteRecord[0] = Eigen::MatrixXf::Zero(n,Horizon);
    xOverWriteRecord[1] = Eigen::MatrixXf::Zero(n,Horizon);

    uOverWriteRecord[0] = Eigen::MatrixXf::Zero(n,Horizon-1);
    uOverWriteRecord[1] = Eigen::MatrixXf::Zero(n,Horizon-1);
    
    
    //rolloutshot
    RolloutShot(0,Horizon-1, xOverWriteRecord[0], uOverWriteRecord[0], L);
    
    //update cost
    UpdateCost(); 
    
    //compute defect
    
    
    //LQ problem approximation
    LQApproximate();
    
    //Backforward Iteration: find S_N S_n l_n and L_N 
    BackwardIteration(); 
    
    
    //update delta x and delta u
    ComputeStateAndControl();
    
    
    
    //line Search:
        // 1.compute cost
        // 2.update the full control and state
        // 3.reset the defect
        // 4.rolloutshot
        // 5.update cost
        // 6.compute the norm of the  delta u and the delta x
        // 7.compute J diff
    
    RolloutShot(0,Horizon-1, xOverWriteRecord[1], uOverWriteRecord[1], L);
}

void FlightControl::mpc::RolloutShot(int startIndex, int endIndex,
                                     Eigen::MatrixXf &xTraj,
                                     Eigen::MatrixXf &uTraj,
                                     Eigen::MatrixXf &xReference,
                                     Eigen::MatrixXf &stateOverWrite,
                                     Eigen::MatrixXf &inputOverWrite,
                                     std::vector<Eigen::MatrixXf> &L){
    stateOverWrite = Eigen::MatrixXf::Zero(stateDim, Horizon);
    inputOverWrite = Eigen::MatrixXf::Zero(controlDim, Horizon-1);
    
    for(int i = startIndex; i <= endIndex; i+=shootingInterval){
        RolloutSingleShot(i,xTraj, uTraj, xReference, stateOverWrite, inputOverWrite, L);

        Eigen::VectorXf dBuf = ComputeSingleDefect();
        d.block(0,i,stateDim,i + shootingInterval-1) = dBuf;
    }
}


void FlightControl::mpc::RolloutSingleShot(int n, 
                                           Eigen::MatrixXf &xTraj,
                                           Eigen::MatrixXf &uTraj,
                                           Eigen::MatrixXf &xReference,
                                           Eigen::MatrixXf &stateOverWrite,
                                           Eigen::MatrixXf &inputOverWrite,
                                           std::vector<Eigen::MatrixXf> &L){
    int endIndex = n + shootingInterval - 1;
    Eigen::MatrixXf uOverWriteBuf;
    if(endIndex >= Horizon){
        endIndex = Horizon -1;
        uOverWriteBuf = Eigen::MatrixXf::Zero(controlDim, shootingInterval-1);
    }
    else
        uOverWriteBuf = Eigen::MatrixXf::Zero(controlDim, shootingInterval);
    
    Eigen::MatrixXf xOverWriteBuf = Eigen::MatrixXf::Zero(stateDim, shootingInterval);
    xOverWriteBuf.block(0,0,stateDim,1) = xTraj.block(0,n,stateDim,1);
    
    for(int i = n; i < endIndex; i++){
        int shiftIndex = i - n;
        if(i>n)
            xOverWriteBuf.block(0,shiftIndex,stateDim,1) = xOverWriteBuf.block(0,shiftIndex-1,stateDim,1);
        
        uOverWriteBuf.block(0,shiftIndex,controlDim,1) =
            uTraj.block(0,i,controlDim,1) + L[i] * (xOverWriteBuf.block(0,shiftIndex,stateDim,1) - xReference.block(0,i,stateDim,1));
       
        //RK45;
        Eigen::MatrixXf xBuf;
        
        xBuf = RK45(Dynamic, (float)(i*simulationDeltaT), 
                    (float)((i+1)*simulationDeltaT),
                    xOverWriteBuf.block(0,shiftIndex,stateDim,1),
                    uOverWriteBuf.block(0,shiftIndex,controlDim,1));

        xOverWriteBuf.block(0,shiftIndex,stateDim,1) = xBuf.block(0,end,stateDim,1);

        if(i == Horizon - 1)
            xOverWriteBuf.block(0,shiftIndex+1,stateDim,1) = xOverWriteBuf.block(0,shiftIndex,stateDim,1);
        
    }


}



Eigen::MatrixXf FlightControl::mpc::RK45(Eigen::MatrixXf (*f)(Eigen::VectorXf x,Eigen::VectorXf u),
                                        float tStart,
                                        float tEnd,
                                        Eigen::VectorXf x,
                                        Eigen::VectorXf u){
    
}


Eigen::MatrixXf FlightControl::mpc::Dynamic(Eigen::VectorXf x,Eigen::VectorXf u){
    float xPosition = x[0];
    float yPosition = x[1];
    float zPosition = x[2];
    float vx = x[3];
    float vy = x[4];
    float vz = x[5];

    float phi = u[0];
    float theta = u[1];
    float psi = u[2];
    float thrust = u[3];

    float ax = (cos(phi) * sin(theta)*cos(psi) + sin(phi) * sin(phi))* thrust / m;
    float ay = (cos(phi) * sin(theta)*sin(psi) - sin(phi) * cos(psi))* thrust / m;
    float az = (cos(phi) * cos(theta))* thrust / m - 9.81;

    vx = vx + ax * simulationDeltaT;
    vy = vy + ay * simulationDeltaT;
    vz = vz + az * simulationDeltaT;

    xPosition = xPosition + vx * simulationDeltaT;
    yPosition = yPosition + vy * simulationDeltaT;
    zPosition = zPosition + vz * simulationDeltaT;

    Eigen::VectorXf res(6);
    res[0] = xPosition; res[1] = yPosition; res[2] = zPosition;
    res[3] = vx;    res[4] = vy;    res[5] = vz;
    
    return res;
}






