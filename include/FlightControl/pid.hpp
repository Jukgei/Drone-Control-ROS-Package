#ifndef PID_H
#define PID_H


#include <iostream>
#include <chrono>

using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

namespace FlightControl{

class pid
{
public:
    pid( float Kp, float Ki, float Kd, float Max, float Min ); 

    float PidOutput(float SetPoint,float Input);
    float clipping();


private:
    high_resolution_clock::time_point StartTime;
    high_resolution_clock::time_point EndTime;
    milliseconds TimeInterval;  
    
    float Error;
    float PastError;
    float IntegralError;
    float DifferentialError;

    float Output;

    float Input;
    float SetPoint; 
    
    float Kp;
    float Ki;
    float Kd;
    
    float MaxLimit;
    float MinLimit;
    
};

}

#endif
