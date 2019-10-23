#include "../include/FlightControl/flightcontrol.hpp"
#include "../include/FlightControl/pid.hpp"
#include <thread>
#include <iostream>
#include <vector>
#include <unistd.h>



#include "FlightControl/state.h"
#include "FlightControl/opticalflow.h"
//using namespace FlightControl;


FlightControl::FlightControlNode::FlightControlNode(ros::NodeHandle &n){
    
    FlightStatus = 255;

    DisplayMode = 255;

    this->InitSubcribers(n);

    this->InitPublishers(n);

    this->InitServices(n);

    this->InitFlightControlThread();

}

void FlightControl::FlightControlNode::InitSubcribers(ros::NodeHandle &n){

    AttitudeSubscriber =n.subscribe<geometry_msgs::QuaternionStamped>
        ("dji_sdk/attitude",10, &FlightControl::FlightControlNode::GetQuaternionCallBack,this);

    FlightStatusSubscriber =n.subscribe<std_msgs::UInt8>
        ("dji_sdk/FlightStatus", 10, &FlightControl::FlightControlNode::GetFlightStatusCallBack,this);

    DisplayModeSubscriber =n.subscribe<std_msgs::UInt8>
        ("dji_sdk/DisplayMode", 10, &FlightControl::FlightControlNode::GetDisplayModeCallBack,this);

    HorizontalVelocitySubscriber = n.subscribe<geometry_msgs::Vector3Stamped>
        ("dji_sdk/velocity", 10, &FlightControl::FlightControlNode::GetVelocityCallBack,this);

    GpsHeightSubscriber = n.subscribe<sensor_msgs::NavSatFix>
        ("dji_sdk/gps_position", 10, &FlightControl::FlightControlNode::GetGpsHeightCallBack,this);

    //From radar
    HeightSubscriber = n.subscribe<FlightControl::state>
        ("state", 10, &FlightControl::FlightControlNode::GetHeightCallBack,this);

    DeltaPositionSubscriber = n.subscribe<FlightControl::opticalflow>
        ("opticalflow", 10, &FlightControl::FlightControlNode::GetDeltaPositionCallBack,this);
}

void FlightControl::FlightControlNode::InitPublishers(ros::NodeHandle &n){
    this->CtrAttitudePublisher = 
        n.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
    
//    std::thread pub(std::bind(&FlightControlNode::publish,this));
//    pub.detach();
}

void FlightControl::FlightControlNode::InitServices(ros::NodeHandle &n){
    CtrlAuthorityService = n.serviceClient<dji_sdk::SDKControlAuthority>
        ("dji_sdk/sdk_control_authority");

    DroneTaskControlService = n.serviceClient<dji_sdk::DroneTaskControl>
        ("dji_sdk/drone_task_control");

}

void FlightControl::FlightControlNode::InitFlightControlThread(){
    std::thread FlightCtr(std::bind(&FlightControl::FlightControlNode::FlightControlThread,this));
    FlightCtr.detach();        
}

void FlightControl::FlightControlNode::FlightControlThread(){
    //take off
    bool ControlAuthority = ObtainControl();
    if(!ControlAuthority){
        ROS_ERROR("Cannot obtain control");
        return ;
    }
    bool isTakeoff = MonitoredTakeoff();
    if(!isTakeoff){
        ROS_ERROR("Cannot Takeoff");
    }
    ROS_INFO("Control Start");
    
    FlightControl::pid myVxController(0.1,0.01,0.001,10,-10); 
   
    FlightControl::pid myThrustController(2.5, 0.0105, 0.05, 100.0, 0);

    this->HeightAboveTakeoff = HeightGps;


    while(true){
       //run flight control algorithm 
        
        double pitch = myVxController.PidOutput(1,HorizontalVelocity.y) ;
        double thrust = myThrustController.PidOutput(20.0,HeightGps-HeightAboveTakeoff);
        sensor_msgs::Joy controlVelYawRate;
        uint8_t flag = (DJISDK::VERTICAL_THRUST   |
                    DJISDK::HORIZONTAL_ANGLE      |
                    DJISDK::YAW_RATE              |
                    DJISDK::HORIZONTAL_BODY       |
                    DJISDK::STABLE_DISABLE);
        controlVelYawRate.axes.push_back(0);    //roll->Vy
        controlVelYawRate.axes.push_back(pitch);    //pitch->Vx
        controlVelYawRate.axes.push_back(thrust);    //thrust
        controlVelYawRate.axes.push_back(0);    //yawRate
        controlVelYawRate.axes.push_back(flag);
        
        CtrAttitudePublisher.publish(controlVelYawRate);
        
        usleep(10000);
        
    }
    if(TakeoffLand(dji_sdk::DroneTaskControl::Request::TASK_LAND))
        ROS_INFO("Landing success");
}

//void FlightControl::FlightControlNode::Publish(){
//    
//
//}

void FlightControl::FlightControlNode::GetQuaternionCallBack(const geometry_msgs::QuaternionStamped::ConstPtr& msg){
    this->Attitude = msg->quaternion;
}

void FlightControl::FlightControlNode::GetFlightStatusCallBack(const std_msgs::UInt8::ConstPtr& msg){
    this->FlightStatus = msg->data;
}

void FlightControl::FlightControlNode::GetDisplayModeCallBack(const std_msgs::UInt8::ConstPtr& msg){
    this->DisplayMode = msg->data;
}

bool FlightControl::FlightControlNode::MonitoredTakeoff(){

  ros::Time start_time = ros::Time::now();

  if(!TakeoffLand(dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF)) {
    return false;
  }

  ros::Duration(0.01).sleep();
  ros::spinOnce();

  // Step 1.1: Spin the motor
  while (FlightStatus != DJISDK::FlightStatus::STATUS_ON_GROUND &&
         DisplayMode != DJISDK::DisplayMode::MODE_ENGINE_START &&
         ros::Time::now() - start_time < ros::Duration(3)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(4)) {
    ROS_ERROR("Takeoff failed. Motors are not spinnning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Motor Spinning ...");
    ros::spinOnce();
  }


  // Step 1.2: Get in to the air
  while (FlightStatus != DJISDK::FlightStatus::STATUS_IN_AIR &&
          (DisplayMode != DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || DisplayMode != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(2)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if(ros::Time::now() - start_time > ros::Duration(3)) {
    ROS_ERROR("Takeoff failed. Aircraft is still on the ground, but the motors are spinning.");
    return false;
  }
  else {
    start_time = ros::Time::now();
    ROS_INFO("Ascending...");
    ros::spinOnce();
  }

  // Final check: Finished takeoff
  while ( (DisplayMode == DJISDK::DisplayMode::MODE_ASSISTED_TAKEOFF || DisplayMode == DJISDK::DisplayMode::MODE_AUTO_TAKEOFF) &&
          ros::Time::now() - start_time < ros::Duration(2)) {
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }

  if ( DisplayMode != DJISDK::DisplayMode::MODE_P_GPS || DisplayMode != DJISDK::DisplayMode::MODE_ATTITUDE)
  {
    ROS_INFO("Successful takeoff!");
    start_time = ros::Time::now();
  }
  else
  {
    ROS_ERROR("Takeoff finished, but the aircraft is in an unexpected mode. Please connect DJI GO.");
    return false;
  }

  return true;
}

bool FlightControl::FlightControlNode::ObtainControl(){

  dji_sdk::SDKControlAuthority authority;
  authority.request.control_enable=1;
  CtrlAuthorityService.call(authority);

  if(!authority.response.result)
  {
    ROS_ERROR("obtain control failed!");
    return false;
  }

  return true;
}

bool FlightControl::FlightControlNode::TakeoffLand(int task){
    
  dji_sdk::DroneTaskControl droneTaskControl;

  droneTaskControl.request.task = task;

  DroneTaskControlService.call(droneTaskControl);

  if(!droneTaskControl.response.result)
  {
    ROS_ERROR("takeoff_land fail");
    return false;
  }

  return true;

}

void FlightControl::FlightControlNode::GetHeightCallBack(const FlightControl::state::ConstPtr& msg){
    this->Height = msg->height;
    //ROS_INFO("Height is %f\n", this->Height);
}

void FlightControl::FlightControlNode::GetDeltaPositionCallBack(const FlightControl::opticalflow::ConstPtr& msg){
    std::vector<float> temp = msg->displacement;
    if(temp.size() == 2){
        x = temp[0];
        y = temp[1];
    }
    //ROS_INFO("X:%f, Y:%f\n",x,y);
}

void FlightControl::FlightControlNode::GetGpsHeightCallBack(const sensor_msgs::NavSatFix::ConstPtr & msg){
    HeightGps = msg->altitude;
    ROS_INFO("HeightGps: %lf",HeightGps);
}

void FlightControl::FlightControlNode::GetVelocityCallBack(const geometry_msgs::Vector3Stamped::ConstPtr & msg){
    this->HorizontalVelocity = msg->vector;
    //ROS_INFO("Vx:%f, Vy:%f, Vz:%f \n", HorizontalVelocity.y,HorizontalVelocity.x,HorizontalVelocity.z);

}

