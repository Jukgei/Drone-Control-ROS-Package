#include "../include/FlightControl/mpc.hpp"

FlightControl::mpc::mpc(int m, int n, int horizon,float mass, float g,int shootingInterval,
                        Eigen::VectorXf vQ,
                        Eigen::VectorXf vQ_,
                        Eigen::VectorXf vR){
    this->shootingInterval = shootingInterval;
    this->pDynamic = &mpc::Dynamic;
    this->controlDim = m;
    this->stateDim = n;
    this->Horizon = horizon;
    this->Qweight = vQ.diagonal();
    this->Q_  = vQ_.diagonal();
    this->Rweight = vR.diagonal();
    this->m = mass;
    this->g_ = g;
}

Eigen::VectorXf FlightControl::mpc::mpcController(Eigen::VectorXf x, Eigen::VectorXf uPast,
                                                     Eigen::VectorXf xRef){
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

    Eigen::VectorXf res = uOverWrite[2].block(0,0,controlDim,1);
    return res;
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
    RolloutShot(0,Horizon-1,
                xInitTraj,
                uInitTraj,
                xInitTraj,
                xOverWriteRecord[0], uOverWriteRecord[0], L);
    
    //update cost
    //UpdateCost(); 
    //
    ////compute defect
    //
    //
    ////LQ problem approximation
    LQApproximate(xInitTraj,uInitTraj);
    //
    ////Backforward Iteration: find S_N S_n l_n and L_N 
    BackwardIteration(); 
    //
    //
    ////update delta x and delta u
    //
    
    Eigen::MatrixXf diffu = Eigen::MatrixXf::Zero(m,Horizon-1);
    Eigen::MatrixXf diffx = Eigen::MatrixXf::Zero(m,Horizon);
    ComputeStateAndControl(diffu,diffx);

    xInitTraj = xInitTraj + diffx;
    uInitTraj = uInitTraj + diffu;


    RolloutShot(0,Horizon-1,
                xInitTraj,
                uInitTraj,
                xInitTraj,
                xOverWriteRecord[1], uOverWriteRecord[1], L);

    this->xOverWrite = xOverWriteRecord;
    this->uOverWrite = uOverWriteRecord;


} 
void FlightControl::mpc::RolloutShot(int startIndex, int endIndex,
                                     const Eigen::MatrixXf &xTraj,
                                     const Eigen::MatrixXf &uTraj,
                                     const Eigen::MatrixXf &xReference,
                                     Eigen::MatrixXf &stateOverWrite,
                                     Eigen::MatrixXf &inputOverWrite,
                                     std::vector<Eigen::MatrixXf> &L){
    stateOverWrite = Eigen::MatrixXf::Zero(stateDim, Horizon);
    inputOverWrite = Eigen::MatrixXf::Zero(controlDim, Horizon-1);
    
    for(int i = startIndex; i <= endIndex; i+=shootingInterval){
        RolloutSingleShot(i,xTraj, uTraj, xReference, stateOverWrite, inputOverWrite, L);

        Eigen::VectorXf dBuf = ComputeSingleDefect(i,stateOverWrite, xTraj);
        d.block(0,i,stateDim,i + shootingInterval-1) = dBuf;
    }
}

Eigen::VectorXf FlightControl::mpc::ComputeSingleDefect(int n,
                                                        const Eigen::MatrixXf &stateOverWrite,
                                                        const Eigen::MatrixXf &xTraj){
   
    int index = Horizon < (n + shootingInterval) ? Horizon: (n + shootingInterval);
    Eigen::MatrixXf dBuf = Eigen::MatrixXf::Zero(stateDim,shootingInterval);

    if(index < Horizon){
        dBuf = stateOverWrite.block(0,index-1,stateDim, 1) - xTraj.block(0,index,stateDim,1);
    }
    return dBuf;
}

void FlightControl::mpc::RolloutSingleShot(int n, 
                                           const Eigen::MatrixXf &xTraj,
                                           const Eigen::MatrixXf &uTraj,
                                           const Eigen::MatrixXf &xReference,
                                           Eigen::MatrixXf &stateOverWrite,
                                           Eigen::MatrixXf &inputOverWrite,
                                           std::vector<Eigen::MatrixXf> &L){

    int endIndex = n + shootingInterval - 1;
    Eigen::MatrixXf uOverWriteBuf;
    Eigen::MatrixXf xBuf;
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
        xBuf = RK45(pDynamic, 
                    (float)(i*simulationDeltaT), 
                    (float)((i+1)*simulationDeltaT),
                    xOverWriteBuf.block(0,shiftIndex,stateDim,1),
                    uOverWriteBuf.block(0,shiftIndex,controlDim,1));

        
        xOverWriteBuf.block(0,shiftIndex,stateDim,1) = xBuf.block(0,simulationPoint-1,stateDim,1);

        if(i == Horizon - 1)
            xOverWriteBuf.block(0,shiftIndex+1,stateDim,1) = xOverWriteBuf.block(0,shiftIndex,stateDim,1);
        
    }
    int xCols = xBuf.cols();
    int uCols = uOverWriteBuf.cols();
    stateOverWrite.block(0,n,stateDim,xCols) = xBuf;
    inputOverWrite.block(0,n,controlDim,uCols) = uOverWriteBuf;  
}



Eigen::MatrixXf FlightControl::mpc::RK45(Eigen::VectorXf (mpc::*f)(const Eigen::VectorXf &x,const Eigen::VectorXf &u),
                                        float tStart,
                                        float tEnd,
                                        const Eigen::VectorXf &x,
                                        const Eigen::VectorXf &u){

    Eigen::MatrixXf y = Eigen::MatrixXf::Zero(stateDim,simulationPoint);
    y.block(0,0,stateDim,1) = x;
    int loop = (tEnd - tStart )/(simulationPoint-1);
    for(int i = 0; i < loop; i++){
        Eigen::VectorXf k1 = (this->*f)(x,u);
        Eigen::VectorXf k2 = (this->*f)(x + k1*simulationDeltaT/2,u);
        Eigen::VectorXf k3 = (this->*f)(x + k2*simulationDeltaT/2,u);
        Eigen::VectorXf k4 = (this->*f)(x + k3*simulationDeltaT,u);
        y.block(0,i+1,stateDim,1) = y.block(0,i,stateDim,1) + (k1 + 2*k2 + 2*k3 + k4)/6 *simulationDeltaT;
    }
    return y;
}


Eigen::VectorXf FlightControl::mpc::Dynamic(const Eigen::VectorXf &x,const Eigen::VectorXf &u){
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


void FlightControl::mpc::LQApproximate(const Eigen::MatrixXf &xTraj,
                                       const Eigen::MatrixXf &uTraj){
    
    Eigen::MatrixXf QCost = this->Qweight;
    Eigen::MatrixXf RCost = this->Rweight;

    Eigen::MatrixXf QCost_ = this->Q_;

    for(int i = 0; i < Horizon-1; i++){
        Q[i] = (QCost + QCost.transpose())*simulationDeltaT;
        R[i] = (RCost + RCost.transpose())*simulationDeltaT;
        P[i] = Eigen::MatrixXf::Zero(controlDim,stateDim);

        Eigen::MatrixXf xDiff = xTraj.block(0,i,stateDim,1) - xRefTraj.block(0,i,stateDim,1);
        Eigen::MatrixXf uDiff = uTraj.block(0,i,controlDim,1) - uRefTraj.block(0,i,controlDim,1);
       
        q[i] = (xDiff.transpose() * QCost.transpose() + xDiff.transpose() * QCost).transpose() * simulationDeltaT;
        r[i] = (uDiff.transpose() * RCost.transpose() + uDiff.transpose() * RCost).transpose() * simulationDeltaT;
    
        LinearSystem(xTraj.block(0,i,stateDim,1),
                     uTraj.block(0,i,controlDim,1),
                     A[i],B[i]);
    }
     
    Q[Horizon-1] = (QCost + QCost.transpose())*simulationDeltaT;
    R[Horizon-1] = (RCost + RCost.transpose())*simulationDeltaT;
    P[Horizon-1] = Eigen::MatrixXf::Zero(controlDim,stateDim);

    Eigen::MatrixXf xDiff = xTraj.block(0,Horizon-1,stateDim,1) - xRefTraj.block(0,Horizon-1,stateDim,1);
    Eigen::MatrixXf uDiff = uTraj.block(0,Horizon-1,controlDim,1) - uRefTraj.block(0,Horizon-1,controlDim,1);
       
    q[Horizon-1] = (xDiff.transpose() * QCost.transpose() + xDiff.transpose() * QCost).transpose() * simulationDeltaT;
    r[Horizon-1] = (uDiff.transpose() * RCost.transpose() + uDiff.transpose() * RCost).transpose() * simulationDeltaT;
    
    LinearSystem(xTraj.block(0,Horizon-1,stateDim,1),
                 uTraj.block(0,Horizon-1,controlDim,1),
                 A[Horizon-1],B[Horizon-1]);
}


void FlightControl::mpc::LinearSystem(const Eigen::VectorXf xTraj,
                                      const Eigen::VectorXf uTraj,
                                      Eigen::MatrixXf &A,
                                      Eigen::MatrixXf &B){
    int n = stateDim;
    int m = controlDim;
    for(int i = 0; i < n; i++){
        Eigen::VectorXf xPertubed = xTraj;
        xPertubed[i] += std::numeric_limits<float>::epsilon();
        Eigen::VectorXf xPlus = Dynamic(xPertubed,uTraj);

        xPertubed = xTraj;
        
        xPertubed[i] += std::numeric_limits<float>::epsilon();
        Eigen::VectorXf xMinus = Dynamic(xPertubed,uTraj);
        A.block(0,i,stateDim,1) = (xPlus - xMinus)/ (2 * std::numeric_limits<float>::epsilon());
    }

    for(int i = 0; i < m; i++){
        Eigen::VectorXf uPertubed = uTraj;
        uPertubed[i] += std::numeric_limits<float>::epsilon();
        Eigen::VectorXf uPlus = Dynamic(xTraj,uPertubed);

        uPertubed = uTraj;
        
        uPertubed[i] += std::numeric_limits<float>::epsilon();
        Eigen::VectorXf uMinus = Dynamic(xTraj,uPertubed);
        B.block(0,i,controlDim,1) = (uPlus - uMinus)/ (2 * std::numeric_limits<float>::epsilon());
    }
}

void FlightControl::mpc::BackwardIteration(){
    S[Horizon-1] = Q[Horizon-1];
    s[Horizon-1] = q[Horizon-1];
    for(int i = Horizon - 2; i >= 0; i--){
        h[i] = r[i] + B[i].transpose() * (s[i+1]+ S[i+1]*d.block(0,i,stateDim,1) );
        G[i] = P[i] + B[i].transpose() * S[i+1] * A[i];
        H[i] = R[i] + B[i].transpose() * S[i+1] * B[i];
        
        l[i] = - H[i].llt().solve(h[i]);
        L[i] = - H[i].llt().solve(G[i]);

        S[i] = Q[i] + A[i].transpose() * S[i+1] * A[i] - L[i].transpose()*H[i]*L[i];
        s[i] = q[i] + A[i].transpose() * (s[i+1] + S[i+1] * d.block(0,i,stateDim,1)) + 
            G[i].transpose() * l[i] + L[i].transpose() * (h[i] + H[i] * l[i]);

    }
}

void FlightControl::mpc::ComputeStateAndControl(Eigen::MatrixXf &diffu,
                                                Eigen::MatrixXf &diffx){
    diffx.block(0,0,stateDim,1) = Eigen::MatrixXf::Zero(stateDim,1);

    for(int i = 0; i < Horizon-1; i++){
        diffu.block(0,i,controlDim,1) = l[i] + L[i] * diffx.block(0,i,stateDim,1);
        diffx.block(0,i+1,stateDim,1) = A[i] * diffx.block(0,i,stateDim,1) 
            + B[i] * diffu.block(0,i,stateDim,1) + d.block(0,i,stateDim,1);
    }

}
