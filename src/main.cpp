#include <iostream>
#include "../include/FlightControl/flightcontrol.hpp"
#include <ros/ros.h>



int main( int argc, char **argv ){
    ros::init(argc, argv, "FlightControl");
    ros::NodeHandle n;
    //RoboticArm::RoboticArmNode * myArmNode = new RoboticArm::RoboticArmNode(n);
    FlightControl::FlightControlNode *myFlightControlNode = new FlightControl::FlightControlNode(n);

    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();

    delete myFlightControlNode;
    myFlightControlNode = nullptr;
    
    return 0;

}
