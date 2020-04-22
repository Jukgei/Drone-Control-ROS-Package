#include "../include/FlightControl/mpc.hpp"

FlightControl::mpc::mpc(int m, int n, int horizon,float mass,
                        float g,int shootingInterval,
                        float dt, int simulationPoint,
                        Eigen::VectorXf vQ,
                        Eigen::VectorXf vQ_,
                        Eigen::VectorXf vR){
    this->shootingInterval = shootingInterval;
    this->pDynamic = &mpc::Dynamic;
    this->controlDim = m;
    this->stateDim = n;
    this->Horizon = horizon;
    this->Qweight = vQ.asDiagonal();
    this->Q_  = vQ_.asDiagonal();
    this->Rweight = vR.asDiagonal();
    this->m = mass;
    this->g_ = g;
    this->simulationPoint = simulationPoint;
    this->simulationDeltaT = dt;
    this->d = Eigen::MatrixXf::Zero(n,horizon);
    A = std::vector<Eigen::MatrixXf>(horizon);
    B = std::vector<Eigen::MatrixXf>(horizon);
    Q = std::vector<Eigen::MatrixXf>(horizon);
    R = std::vector<Eigen::MatrixXf>(horizon);
    P = std::vector<Eigen::MatrixXf>(horizon);
    q = std::vector<Eigen::MatrixXf>(horizon);
    r = std::vector<Eigen::MatrixXf>(horizon);
    S = std::vector<Eigen::MatrixXf>(horizon);
    s = std::vector<Eigen::MatrixXf>(horizon);
    
    S = std::vector<Eigen::MatrixXf>(horizon);
    s = std::vector<Eigen::MatrixXf>(horizon);
    h = std::vector<Eigen::MatrixXf>(horizon);
    G = std::vector<Eigen::MatrixXf>(horizon);
    H = std::vector<Eigen::MatrixXf>(horizon);
    l = std::vector<Eigen::MatrixXf>(horizon);
    L = std::vector<Eigen::MatrixXf>(horizon);
    
    std::cout<<"Construct success"<<std::endl;
}

Eigen::VectorXf FlightControl::mpc::mpcController(Eigen::VectorXf x, Eigen::VectorXf uPast,
                                                     Eigen::VectorXf xRef){
    //set xRef and uRef 
    //std::cout<<"I come in 23 lines"<<std::endl;
    this->uRefTraj = Eigen::MatrixXf::Zero(controlDim,Horizon-1);
    uRefTraj.block(3,0,1,Horizon-1) = m*g_*Eigen::MatrixXf::Ones(1,Horizon-1);
    //std::cout<<"uRefTraj is OK"<<std::endl;
    this->xRefTraj = Eigen::MatrixXf::Zero(stateDim,Horizon);   
    //std::cout<<"xRefTraj is OK"<<std::endl;
    for(int i = 0; i < Horizon -1 ;i ++){
        //std::cout<<"I come in 26 lines"<<std::endl;
        xRefTraj.block(0,i,stateDim,1) = xRef;
    }
    //std::cout<<"xRef "<<xRefTraj<<std::endl;
    
    //set xInit and uInit
    this->xInitTraj = Eigen::MatrixXf::Zero(stateDim,Horizon);
    this->uInitTraj = Eigen::MatrixXf::Zero(controlDim,Horizon-1);
    
    //std::cout<<"finish Initialize xInit and u Init"<<std::endl;
    for(int i = 0; i < Horizon ; i ++)
        xInitTraj.block(0,i,stateDim,1) = x;
    
    //std::cout<<"xInitTraj "<<xInitTraj<<std::endl;
    //std::cout<<"finish value xInit"<<std::endl;
    for(int i = 0; i < Horizon-1; i++)
        uInitTraj.block(0,i,controlDim,1) = uPast;
    
    //std::cout<<"uInitTraj "<<uInitTraj<<std::endl;
    //std::cout<<"finish value uInit"<<std::endl;
    //std::cout<<"Ready to GNMS"<<std::endl;
    //GNMS
    GaussNewtonMutipleShooting();

    //std::cout<<"GNMS success"<<std::endl;
    //std::cout<<uOverWrite[1]<<std::endl;
    Eigen::VectorXf res = uOverWrite[1].block(0,0,controlDim,1);
    //std::cout<<"u"<<std::endl;
    //std::cout<<uOverWrite[1].block(0,0,controlDim,12)<<std::endl;
    //std::cout<<"x"<<std::endl;
    //std::cout<<xOverWrite[1].block(0,0,stateDim,12)<<std::endl;
    //std::cout<<"u end"<<std::endl;
    //std::cout<<uOverWrite[1].block(0,80,controlDim,12)<<std::endl;
    //std::cout<<"x end"<<std::endl;
    //std::cout<<xOverWrite[1].block(0,80,stateDim,12)<<std::endl;
    //while(1);
    return res;
}

void FlightControl::mpc::GaussNewtonMutipleShooting(){
    
    int m = controlDim;
    int n = stateDim;

    //Initialize guess point use a trival controller
    //std::vector<Eigen::MatrixXf> L(Horizon);
    for(int i = 0; i < Horizon; i++)
        this->L[i] = Eigen::MatrixXf::Zero(m,n);
    
    
    //std::cout<<"init guess ok"<<std::endl;

    //Initialize x and u result
    std::vector<Eigen::MatrixXf> xOverWriteRecord(2);
    std::vector<Eigen::MatrixXf> uOverWriteRecord(2);
    xOverWriteRecord[0] = Eigen::MatrixXf::Zero(n,Horizon);
    xOverWriteRecord[1] = Eigen::MatrixXf::Zero(n,Horizon);

    uOverWriteRecord[0] = Eigen::MatrixXf::Zero(n,Horizon-1);
    uOverWriteRecord[1] = Eigen::MatrixXf::Zero(n,Horizon-1);
    
   // std::cout<<"init Record ok"<<std::endl;
    
    //rolloutshot
    RolloutShot(0,Horizon-1,
                xInitTraj,
                uInitTraj,
                xInitTraj,
                xOverWriteRecord[0], uOverWriteRecord[0], this->L);
    
    //std::cout<<"xOverWrite"<<std::endl;
    //std::cout<<xOverWriteRecord[0].block(0,0,stateDim,15)<<std::endl;

    //std::cout<<"d"<<this->d<<std::endl;

    //std::cout<<"rolloushot ok"<<std::endl;
    
    //update cost
    //UpdateCost(); 
    //
    ////compute defect
    //
    //
    ////LQ problem approximation
    LQApproximate(xInitTraj,uInitTraj);
    //
    //std::cout<<"LQApproximate ok"<<std::endl;
    ////Backforward Iteration: find S_N S_n l_n and L_N 
    BackwardIteration(); 
    //
    //std::cout<<"Backward Iteration ok"<<std::endl;
    //
    ////update delta x and delta u
    //
    //
    
    Eigen::MatrixXf diffu = Eigen::MatrixXf::Zero(m,Horizon-1);
    Eigen::MatrixXf diffx = Eigen::MatrixXf::Zero(n,Horizon);
    
    //std::cout<<"update delta x and delta u ok"<<std::endl;

    ComputeStateAndControl(diffu,diffx);

    //std::cout<<"compute state and control ok"<<std::endl;
    //std::cout<<"diffu"<<diffu<<std::endl;
    //std::cout<<"diffx"<<diffx<<std::endl;

    xInitTraj = xInitTraj + diffx;
    uInitTraj = uInitTraj + diffu;
    for(int i = 0; i < Horizon -1; i++)
        uInitTraj.block(0,i,controlDim,1) = 
            ConvertUAVControl(uInitTraj.block(0,i,controlDim,1));
    
    //std::cout<<"diffu"<<std::endl;
    //std::cout<<diffu.block(0,0,controlDim,10)<<std::endl;
    //std::cout<<"xTraj and uTraj update ok"<<std::endl;

    RolloutShot(0,Horizon-1,
                xInitTraj,
                uInitTraj,
                xInitTraj,
                xOverWriteRecord[1], uOverWriteRecord[1], this->L);

    //std::cout<<"changex"<<xOverWriteRecord[1] - xInitTraj<<std::endl;
    //std::cout<<"changeu"<<uOverWriteRecord[1] - uInitTraj<<std::endl;
    //std::cout<<"rollout shot ok"<<std::endl;
    
    this->xOverWrite = xOverWriteRecord;
    //std::cout<<"xOverWrite"<<std::endl;
    //std::cout<<xOverWrite[1].block(0,0,stateDim,10)<<std::endl;
    this->uOverWrite = uOverWriteRecord;
    
    //std::cout<<"GNMS ok"<<std::endl;

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
    
    for(int i = startIndex; i < endIndex; i+=shootingInterval){
        //std::cout<<"i:"<<i<<std::endl;
        RolloutSingleShot(i,xTraj, uTraj, xReference, stateOverWrite, inputOverWrite, L);
        //std::cout<<"Single shot!"<<std::endl;
        Eigen::VectorXf dBuf = ComputeSingleDefect(i,stateOverWrite, xTraj);
        //std::cout<<"Compute Single Defect"<<std::endl;
        //std::cout<<"dBuf"<<dBuf<<std::endl;
        //std::cout<<"dBlock"<<d.block(0,i,stateDim,shootingInterval)<<std::endl;
        d.block(0,i,stateDim,shootingInterval) = dBuf;
    }
    //std::cout<<"input"<<std::endl;
    //std::cout<<inputOverWrite<<std::endl;
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
    Eigen::MatrixXf xOverWriteBuf;
    Eigen::MatrixXf xBuf;
    int uTrajSize;
    int xTrajSize;
    if(endIndex >= Horizon-1){
        uTrajSize = endIndex - Horizon + 1;
        xTrajSize = uTrajSize + 1;
        uOverWriteBuf = Eigen::MatrixXf::Zero(controlDim, uTrajSize); 
        xOverWriteBuf = Eigen::MatrixXf::Zero(stateDim, xTrajSize); 
        endIndex = Horizon -2;
    }
    else{
        uTrajSize = shootingInterval;
        xTrajSize = shootingInterval;
        uOverWriteBuf = Eigen::MatrixXf::Zero(controlDim, shootingInterval);
        xOverWriteBuf = Eigen::MatrixXf::Zero(stateDim,shootingInterval);
    }
   
    //std::cout<<"uOverWriteBuf finished"<<std::endl;
    //std::cout<<"xTraj(149)"<<std::endl;
    //std::cout<<xTraj.block(0,n,stateDim,1)<<std::endl;
    xOverWriteBuf.block(0,0,stateDim,1) = xTraj.block(0,n,stateDim,1);
   
    //std::cout<<"xOverWriteBuf finished"<<std::endl;
    for(int i = n; i <= endIndex; i++){
        int shiftIndex = i - n;
        if(i>n)
            xOverWriteBuf.block(0,shiftIndex,stateDim,1) = xOverWriteBuf.block(0,shiftIndex-1,stateDim,1);
        
        //std::cout<<(xOverWriteBuf.block(0,shiftIndex,stateDim,1) - xReference.block(0,i,stateDim,1))<<std::endl;
        //std::cout<<"Ready to compute uOverWritedBuf"<<std::endl;
        uOverWriteBuf.block(0,shiftIndex,controlDim,1) =
            uTraj.block(0,i,controlDim,1) + L[i] * (xOverWriteBuf.block(0,shiftIndex,stateDim,1) - xReference.block(0,i,stateDim,1));
         
        //std::cout<<"front"<<std::endl;
        //std::cout<<uOverWriteBuf.block(0,shiftIndex,controlDim,1)<<std::endl;
        uOverWriteBuf.block(0,shiftIndex,controlDim,1) = 
            ConvertUAVControl(uOverWriteBuf.block(0,shiftIndex,controlDim,1));
        //std::cout<<"black"<<std::endl;
        //std::cout<<uOverWriteBuf.block(0,shiftIndex,controlDim,1)<<std::endl;

        //std::cout<<"ready to RK45"<<std::endl;
        //RK45;
        xBuf = RK45(pDynamic, 
                    (float)(i*simulationDeltaT), 
                    (float)((i+1)*simulationDeltaT),
                    xOverWriteBuf.block(0,shiftIndex,stateDim,1),
                    uOverWriteBuf.block(0,shiftIndex,controlDim,1));
        //std::cout<<"xBuf"<<std::endl;
        //std::cout<<xBuf<<std::endl;
        
        xOverWriteBuf.block(0,shiftIndex,stateDim,1) = xBuf.block(0,simulationPoint,stateDim,1);

        if(i == Horizon - 1)
            xOverWriteBuf.block(0,shiftIndex+1,stateDim,1) = xOverWriteBuf.block(0,shiftIndex,stateDim,1);
        
    }
    
    //std::cout<<"jump out the loop"<<std::endl;
    int xCols = xBuf.cols();
    int uCols = uOverWriteBuf.cols();

    //std::cout<<"xCols:"<<xCols<<std::endl;
    //std::cout<<"uCols:"<<uCols<<std::endl;
    //std::cout<<"xBuf:"<<xBuf<<std::endl;
    //std::cout<<"uOverwrite"<<uOverWriteBuf<<std::endl;
    stateOverWrite.block(0,n,stateDim,xTrajSize) = xOverWriteBuf;
    //std::cout<<"xBuf ok!"<<std::endl;
    inputOverWrite.block(0,n,controlDim,uTrajSize) = uOverWriteBuf;  
    //std::cout<<"uOverWriter OK!"<<std::endl;
}



Eigen::MatrixXf FlightControl::mpc::RK45(Eigen::VectorXf (mpc::*f)(const Eigen::VectorXf &x,const Eigen::VectorXf &u),
                                        float tStart,
                                        float tEnd,
                                        const Eigen::VectorXf &x,
                                        const Eigen::VectorXf &u){

    Eigen::MatrixXf y = Eigen::MatrixXf::Zero(stateDim,simulationPoint+1);
    //std::cout<<"y init success"<<std::endl;
    //std::cout<<"x"<<x<<std::endl;
    ////std::cout<<"y"<<y.block(0,0,stateDim,1)<<x<<std::endl;
    //std::cout<<"yhhhhh"<<std::endl;
    //std::cout<<'y'<<y.cols()<<"y row"<<y.rows()<<std::endl;
    y.block(0,0,stateDim,1) = x;
    //std::cout<<"y value success"<<std::endl;
    //int loop = (tEnd - tStart )/(simulationPoint-1);
    //std::cout<<"RK45 loop start"<<std::endl;
    for(int i = 0; i < simulationPoint; i++){
        Eigen::VectorXf k1 = (this->*f)(x,u);
        Eigen::VectorXf k2 = (this->*f)(x + k1*simulationDeltaT/2,u);
        Eigen::VectorXf k3 = (this->*f)(x + k2*simulationDeltaT/2,u);
        Eigen::VectorXf k4 = (this->*f)(x + k3*simulationDeltaT,u);
        //std::cout<<"k1 is"<<k1<<std::endl;
        //std::cout<<"k2 is"<<k2<<std::endl;
        //std::cout<<"k3 is "<<k3<<std::endl;
        //std::cout<<"k4 is"<<k4<<std::endl;
        y.block(0,i+1,stateDim,1) = y.block(0,i,stateDim,1) + (k1 + 2*k2 + 2*k3 + k4)/6 *simulationDeltaT;
    }
    //std::cout<<"y is "<<y<<std::endl;
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
    
    float ax = (cos(phi) * sin(theta)*cos(psi) + sin(phi) * sin(psi))* thrust / m;
    float ay = (cos(phi) * sin(theta)*sin(psi) - sin(phi) * cos(psi))* thrust / m;
    float az = (cos(phi) * cos(theta))* thrust / m - g_;

    //vx = vx + ax * simulationDeltaT;
    //vy = vy + ay * simulationDeltaT;
    //vz = vz + az * simulationDeltaT;

    //xPosition = xPosition + vx * simulationDeltaT;
    //yPosition = yPosition + vy * simulationDeltaT;
    //zPosition = zPosition + vz * simulationDeltaT;

    Eigen::VectorXf res(6);
    //res[0] = xPosition; res[1] = yPosition; res[2] = zPosition;
    //res[3] = vx;        res[4] = vy;        res[5] = vz;
    res[0] = vx;        res[1] = vy;        res[2] = vz;
    res[3] = ax;        res[4] = ay;        res[5] = az;
    //std::cout<<"out"<<std::endl;
    //std::cout<<res<<std::endl;
    return res;
}


void FlightControl::mpc::LQApproximate(const Eigen::MatrixXf &xTraj,
                                       const Eigen::MatrixXf &uTraj){
    
    Eigen::MatrixXf QCost = this->Qweight;
    Eigen::MatrixXf RCost = this->Rweight;

    Eigen::MatrixXf QCost_ = this->Q_;
    Eigen::MatrixXf xDiff;
    Eigen::MatrixXf uDiff;
    //std::cout<<"Init weight matrix ok"<<std::endl;
    for(int i = 0; i < Horizon-1; i++){
        //std::cout<<"LQApproximate i:"<< i<< std::endl;
        //std::cout<<"QCost"<<std::endl;
        //std::cout<<(QCost+ QCost.transpose())*simulationDeltaT<< std::endl;
        Q[i] = (QCost + QCost.transpose())*simulationDeltaT;
        R[i] = (RCost + RCost.transpose())*simulationDeltaT;
        P[i] = Eigen::MatrixXf::Zero(controlDim,stateDim);
        
        //std::cout<<"Q,R,P,matrix is ok"<< std::endl;
        //std::cout<<"Xtraj block"<< std::endl;
        //std::cout<<xTraj.block(0,i,stateDim,1)<<std::endl;
        //std::cout<<"XRef block"<< std::endl;
        //std::cout<<- xRefTraj.block(0,i,stateDim,1)<<std::endl; 
        xDiff = xTraj.block(0,i,stateDim,1) - xRefTraj.block(0,i,stateDim,1);
        uDiff = uTraj.block(0,i,controlDim,1) - uRefTraj.block(0,i,controlDim,1);
         
        //std::cout<<"xDiff"<<xDiff <<std::endl;
        //std::cout<<"uDiff"<<uDiff <<std::endl;
        //std::cout<<"xDiff and uDiff is ok"<< std::endl;
        
        q[i] = (xDiff.transpose() * QCost.transpose() + xDiff.transpose() * QCost).transpose() * simulationDeltaT;
        r[i] = (uDiff.transpose() * RCost.transpose() + uDiff.transpose() * RCost).transpose() * simulationDeltaT;
        //std::cout<<"r[i]"<<std::endl;   
        //std::cout<<r[i]<<std::endl;   
        //std::cout<<"q and r matrix is ok"<< std::endl;
        LinearSystem(xTraj.block(0,i,stateDim,1),
                     uTraj.block(0,i,controlDim,1),
                     A[i],B[i]);
        //std::cout<<"A[i]"<<A[i]<<std::endl;
        //std::cout<<"B[i]"<<B[i]<<std::endl;

        //std::cout<<"linear system is ok"<< std::endl;
    }
     
    //std::cout<<"Q,R,P,matrix is ok(Horizon)"<< std::endl;
    Q[Horizon-1] = (QCost_ + QCost_.transpose())*simulationDeltaT;
    R[Horizon-1] = (RCost + RCost.transpose())*simulationDeltaT;
    P[Horizon-1] = Eigen::MatrixXf::Zero(controlDim,stateDim);

    //std::cout<<"xDiff and uDiff is ok(Horizon)"<< std::endl;
       
    //std::cout<<"q and r matrix is ok(Horzion)"<< std::endl;
    q[Horizon-1] = (xDiff.transpose() * QCost_.transpose() + xDiff.transpose() * QCost_).transpose() * simulationDeltaT;
    r[Horizon-1] = (uDiff.transpose() * RCost.transpose() + uDiff.transpose() * RCost).transpose() * simulationDeltaT;
    
    //std::cout<<"q and r matrix is ok(Horzion)"<< std::endl;
    LinearSystem(xTraj.block(0,Horizon-1,stateDim,1),
                 uTraj.block(0,Horizon-2,controlDim,1),
                 A[Horizon-1],B[Horizon-1]);

    //std::cout<<"LQApproximate is finish"<< std::endl;
}


void FlightControl::mpc::LinearSystem(const Eigen::VectorXf xTraj,
                                      const Eigen::VectorXf uTraj,
                                      Eigen::MatrixXf &A,
                                      Eigen::MatrixXf &B){
    int n = stateDim;
    int m = controlDim;
    A = Eigen::MatrixXf::Zero(n,n);
    B = Eigen::MatrixXf::Zero(n,m);
    for(int i = 0; i < n; i++){
        Eigen::VectorXf xPertubed = xTraj;
        float disturbX = minDisturb(xPertubed[i]);
        //std::cout<<"xPertubed pre"<< std::endl;
        //std::cout<<xPertubed<<std::endl;
        xPertubed[i] += disturbX;
        //std::cout<<"xPertubed aft"<< std::endl;
        //std::cout<<xPertubed<<std::endl;
        //std::cout<<"disturb add"<< std::endl;
        Eigen::VectorXf xPlus = Dynamic(xPertubed,uTraj);

        //std::cout<<"dynamic is ok"<< std::endl;
        xPertubed = xTraj;
        xPertubed[i] -= disturbX;
        Eigen::VectorXf xMinus = Dynamic(xPertubed,uTraj);
        //std::cout<<"xMinus is ok"<< std::endl;
        Eigen::VectorXf deltaX = xPlus - xMinus;
        for(int j = 0; j < stateDim; j++){
            deltaX[j] = deltaX[j] / (2*disturbX);
        }
        //std::cout<<"delta disturb is "<<std::endl;
        //std::cout<<xPlus - xMinus<<std::endl;
        //std::cout<<"minDelta"<<std::endl;
        //std::cout<<deltaX<<std::endl;
        //while(1);
        A.block(0,i,stateDim,1) = deltaX;
    }

    for(int i = 0; i < m; i++){
        //std::cout<<"Bi"<<i<<std::endl;
        Eigen::VectorXf uPertubed = uTraj;
        float disturbU = minDisturb(uPertubed[i]);
        //std::cout<<"xTraj"<<std::endl;
        //std::cout<<xTraj<<std::endl;
        uPertubed[i] += disturbU;
        //std::cout<<"uPertubed aft add"<< std::endl;
        //std::cout<<uPertubed<< std::endl;
        //std::cout<<"u disturb is ok"<< std::endl;
        Eigen::VectorXf uPlus = Dynamic(xTraj,uPertubed);
        //std::cout<<"uPlus"<< std::endl;
        //std::cout<<uPlus<< std::endl;
        //
        //std::cout<<"xTraj"<< std::endl;
        //std::cout<<xTraj<< std::endl;
        //std::cout<<"u dynamic is ok"<< std::endl;
        uPertubed = uTraj;
        uPertubed[i] -= disturbU;
        //std::cout<<"uPertubed aft add"<< std::endl;
        //std::cout<<"uPertubed aft sub"<< std::endl;
        //std::cout<<uPertubed<< std::endl;
        Eigen::VectorXf uMinus = Dynamic(xTraj,uPertubed);
        Eigen::VectorXf uDelta = uPlus - uMinus;
        for(int j = 0; j < stateDim; j ++){
            uDelta[j] = uDelta[j] /(2*disturbU);
        }
        //std::cout<<"uMinus"<< std::endl;
        //std::cout<<uMinus<< std::endl;
        //std::cout<<"uMinus is ok"<< std::endl;
        //std::cout<<"uPlus - uMinus"<< std::endl;
        //std::cout<<(uPlus - uMinus)<<std::endl;
        //if(i == 3){
        //    std::cout<<"uPlus - uMinus"<<std::endl;
        //    std::cout<<(uPlus - uMinus)<<std::endl;
        //    while(1);
        //}
        B.block(0,i,stateDim,1) = uDelta;
        //B.block(0,i,stateDim,1) =(1/(std::numeric_limits<float>::epsilon())) * (uPlus);
    }
    //    std::cout<<"A"<<std::endl;
    //    std::cout<<A<<std::endl;
    //    std::cout<<"B"<<std::endl;
    //    std::cout<<B<<std::endl;
    //    while(1);
    B *= simulationDeltaT;
    A = A*simulationDeltaT + Eigen::MatrixXf::Identity(stateDim,stateDim);
    //std::cout<<"A"<<A<<std::endl;
    //std::cout<<"B"<<B<<std::endl;
}

float FlightControl::mpc::minDisturb(float a)
{
    float xp = std::abs(a);
    float x1 = std::nextafter(xp,xp+1.0f);
    return (x1 -xp);
}

void FlightControl::mpc::BackwardIteration(){
    S[Horizon-1] = Q[Horizon-1];
    s[Horizon-1] = q[Horizon-1];
    for(int i = Horizon - 2; i >= 0; i--){
        h[i] = r[i] + B[i].transpose() * (s[i+1]+ S[i+1]*d.block(0,i,stateDim,1) );
        G[i] = P[i] + B[i].transpose() * S[i+1] * A[i];
        H[i] = R[i] + B[i].transpose() * S[i+1] * B[i];
        //std::cout<<"r"<<r[i]<<std::endl;
        //std::cout<<"P"<<P[i]<<std::endl;
        //std::cout<<"R"<<R[i]<<std::endl;
        //std::cout<<"h"<<h[i]<<std::endl;
        //std::cout<<"G"<<G[i]<<std::endl;
        //std::cout<<"H"<<H[i]<<std::endl;
        //std::cout<<"A"<<A[i]<<std::endl;
        //std::cout<<"B"<<B[i]<<std::endl;
        
        l[i] = - H[i].llt().solve(h[i]);
        L[i] = - H[i].llt().solve(G[i]);
        //std::cout<<"l"<<l[i]<<std::endl;
        //std::cout<<"L"<<L[i]<<std::endl;
        //if(i == Horizon -3)
        //    while(1);
        S[i] = Q[i] + A[i].transpose() * S[i+1] * A[i] - L[i].transpose()*H[i]*L[i];
        s[i] = q[i] + A[i].transpose() * (s[i+1] + S[i+1] * d.block(0,i,stateDim,1)) + 
            G[i].transpose() * l[i] + L[i].transpose() * (h[i] + H[i] * l[i]);

        //std::cout<<"S"<<S[i]<<std::endl;
        //std::cout<<"s"<<s[i]<<std::endl;
    }
}

void FlightControl::mpc::ComputeStateAndControl(Eigen::MatrixXf &diffu,
                                                Eigen::MatrixXf &diffx){

    //std::cout<<"diffx first col"<<std::endl;
    //std::cout<<diffx.block(0,0,stateDim,1)<<std::endl;
    //std::cout<<"zeros "<<std::endl;
    //std::cout<<Eigen::MatrixXf::Zero(stateDim,1)<<std::endl;
    diffx.block(0,0,stateDim,1) = Eigen::MatrixXf::Zero(stateDim,1);
    
    //std::cout<<"initial diffx"<< std::endl;
    for(int i = 0; i < Horizon-1; i++){
        //std::cout<<"Loop i:"<<i <<std::endl;
        diffu.block(0,i,controlDim,1) = l[i] + L[i] * diffx.block(0,i,stateDim,1);
        //std::cout<<"update u"<<std::endl;
        diffx.block(0,i+1,stateDim,1) = A[i] * diffx.block(0,i,stateDim,1) 
            + B[i] * diffu.block(0,i,controlDim,1) + d.block(0,i,stateDim,1);
        //std::cout<<"update x"<<std::endl;
    }

}

Eigen::VectorXf FlightControl::mpc::UAVConstraint(Eigen::VectorXf control){

    Eigen::VectorXf res = control;
    
    res[0] = res[0] > 10  ?  10:res[0];
    res[0] = res[0] <-10  ?  -10:res[0];

    res[1] = res[1] > 10  ?  10:res[1];
    res[1] = res[1] <-10  ?  -10:res[1];
    
    res[2] = res[2] >  10 ?  10:res[2];
    res[2] = res[2] < -10 ? -10:res[2];
    
    res[3] = res[3] < 0  ?   0: res[3];
    res[3] = res[3] * 2.5f;
    res[3] = res[3] > 100 ? 90: res[3];
    
    return res;
}

Eigen::VectorXf FlightControl::mpc::ConvertUAVControl(Eigen::VectorXf a){
    Eigen::VectorXf c = a;

    c[0] = c[0] - ((int) (c[0] / (2*PI) )) * 2 * PI;
    c[1] = c[1] - ((int) (c[1] / (2*PI) )) * 2 * PI;
    c[2] = c[2] - ((int) (c[2] / (2*PI) )) * 2 * PI;
   
    c[0] = c[0] > PI ? (c[0] - 2 * PI): c[0];
    c[1] = c[1] > PI ? (c[1] - 2 * PI): c[1];
    c[2] = c[2] > PI ? (c[2] - 2 * PI): c[2];
    

    c[0] = c[0] < -PI ? (c[0] + 2 * PI): c[0];
    c[1] = c[1] < -PI ? (c[1] + 2 * PI): c[1];
    c[2] = c[2] < -PI ? (c[2] + 2 * PI): c[2];
    
    //std::cout<<"a"<<std::endl;
    //std::cout<<a<<std::endl;
    //std::cout<<"c"<<std::endl;
    //std::cout<<c<<std::endl;
    return c;
}
