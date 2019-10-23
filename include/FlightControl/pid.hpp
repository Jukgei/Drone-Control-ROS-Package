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
    pid( double Kp, double Ki, double Kd, double Max, double Min ); 

    double PidOutput(double SetPoint,double Input);
    double clipping();


private:
    high_resolution_clock::time_point StartTime;
    high_resolution_clock::time_point EndTime;
    milliseconds TimeInterval;  
    
    double Error;
    double PastError;
    double IntegralError;
    double DifferentialError;

    double Output;

    double Input;
    double SetPoint; 
    
    double Kp;
    double Ki;
    double Kd;
    
    double MaxLimit;
    double MinLimit;
    
};

}

#endif
