#include "../include/FlightControl/flightcontrol.hpp"
#include "../include/FlightControl/pid.hpp"
#include "../include/FlightControl/mpc.hpp"
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

    LocalPositionSubscriber = n.subscribe<geometry_msgs::PointStamped>
        ("dji_sdk/local_position", 10, &FlightControl::FlightControlNode::GetLocalPositionCallBack,this);
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

    SetLocalPositionRefService = n.serviceClient<dji_sdk::SetLocalPosRef> 
        ("dji_sdk/set_local_pos_ref");
}

void FlightControl::FlightControlNode::InitFlightControlThread(){
    std::thread FlightCtr(std::bind(&FlightControl::FlightControlNode::FlightControlThread,this));
    FlightCtr.detach();        
}

void FlightControl::FlightControlNode::FlightControlThread(){
    //take off
    if(!setLocalPosition()){
        ROS_ERROR("GPS health insufficient");
    }
    bool ControlAuthority = ObtainControl();
    if(!ControlAuthority){
        ROS_ERROR("Cannot obtain control");
        return ;
    }
    bool isTakeoff = MonitoredTakeoff();
    if(!isTakeoff){
        ROS_ERROR("Cannot Takeoff");
    }
    //ROS_INFO("Control Start");
    
    //FlightControl::pid myVxController(0.1,0.01,0.001,10,-10); 
   
    //FlightControl::pid myThrustController(2.5, 0.0105, 0.005, 100.0, 0);

    this->HeightAboveTakeoff = HeightGps;
   
    Eigen::VectorXf vQ(6); 
    vQ[0] = 0.01; vQ[1] = 0.01; vQ[2] = 0.01;
    vQ[3] = 0.005; vQ[4] = 0.005; vQ[5] = 0.005;
    Eigen::VectorXf vQ_(6);
    vQ_[0] = 1; vQ_[1] = 1; vQ_[2] = 1;
    vQ_[3] = 0.5;  vQ_[4] = 0.5;  vQ_[5] = 0.5;
    Eigen::VectorXf vR(4);
    vR[0] = 0.05;vR[1] = 0.05;vR[2] = 0.05;vR[3] = 0.001;

    FlightControl::mpc myMpcController(4,6,100,3.3,9.81,1,0.06,5,
                                       vQ,vQ_,vR);

    Eigen::VectorXf uPast(4);
    uPast[0] = 0; uPast[1] = 0; uPast[2] = 0; uPast[3] = 3.3 * 9.81;
    Eigen::VectorXf xRef(6);
    xRef[0] = 0; xRef[1] = 0; xRef[2] = 20; 
    xRef[3] = 0; xRef[4] = 0; xRef[5] = 0;

    Eigen::VectorXf x(6);
    x[0] = localPoint.x;         x[1] = localPoint.y;         x[2] = localPoint.z;
    x[3] = HorizontalVelocity.x; x[4] = HorizontalVelocity.y; x[5] = HorizontalVelocity.z;
    
    std::cout<<"Init x and u success"<<std::endl;
    //std::cout<<x<<std::endl;
    ros::Rate LoopRate(15);
    while(ros::ok()){
       //run flight control algorithm 
        
        //double pitch = myVxController.PidOutput(1,HorizontalVelocity.y) ;
        //double thrust = myThrustController.PidOutput(20.0,HeightGps-HeightAboveTakeoff);
        
        high_resolution_clock::time_point beginTime = high_resolution_clock::now();     //Debug 
        
        //float pidThrust = myThrustController.PidOutput(10,localPoint.z)+ 20;
        Eigen::VectorXf control = myMpcController.mpcController(x,uPast,xRef); 
        //Eigen::VectorXf res = myMpcController.UAVConstraint(control);
        high_resolution_clock::time_point endTime = high_resolution_clock::now();       //Debug
        milliseconds timeInterval = std::chrono::duration_cast<milliseconds>(endTime-beginTime); //Debug
        std::cout<<"Running Time:"<<timeInterval.count() << "ms"<<std::endl;                               //Debug
        std::cout<<"control:";
        //std::cout<<control<<std::endl;
        float roll   = -control[0];
        float pitch  = control[1];
        float yaw    = control[2];
        float thrust = control[3]/(3.3 *(9.81 + 15.25))*100;
        std::cout<<"roll:"<<roll<<
            " pitch:"<<pitch<<
            " yaw:"<<yaw<<
            " thrust"<<thrust<<
            " Real thrust"<<(control[3])<<std::endl;
        sensor_msgs::Joy controlVelYawRate;
        uint8_t flag = (DJISDK::VERTICAL_THRUST   |
                    DJISDK::HORIZONTAL_ANGLE      |
                    DJISDK::YAW_RATE              |
                    DJISDK::HORIZONTAL_BODY       |
                   DJISDK::STABLE_DISABLE);
        controlVelYawRate.axes.push_back(roll);    //roll->Vy
        //controlVelYawRate.axes.push_back(0);    //roll->Vy
        //controlVelYawRate.axes.push_back(10.0/180.0*3.1415926);    //roll->Vy
        controlVelYawRate.axes.push_back(pitch);    //pitch->Vx
        controlVelYawRate.axes.push_back(thrust);    //thrust
        controlVelYawRate.axes.push_back(0);    //yawRate
        controlVelYawRate.axes.push_back(flag);
        
        CtrAttitudePublisher.publish(controlVelYawRate);
       
        x[0] = localPoint.y;         x[1] = localPoint.x;         x[2] = localPoint.z;
        x[3] = HorizontalVelocity.y; x[4] = HorizontalVelocity.x; x[5] = HorizontalVelocity.z;
        // first test one x state.
        // second test roll\pitch and yaw angle;
        //uPast = control;
        //uPast[3] = uPast[3] > 40 ? 40: uPast[3];
        //uPast[3] = control[3];
        std::cout<<"state: "<<"x:"<<x[0]<<
            "  y:"<<x[1]<<
            "  z:"<<x[2]<<
            "  vx:"<<x[3]<<
            "  vy:"<<x[4]<<
            "  vz:"<<x[5]<<std::endl;
        LoopRate.sleep();
        
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

bool FlightControl::FlightControlNode::setLocalPosition()
{
  dji_sdk::SetLocalPosRef localPosReferenceSetter;
  SetLocalPositionRefService.call(localPosReferenceSetter);

  return (bool)localPosReferenceSetter.response.result;
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

void FlightControl::FlightControlNode::GetLocalPositionCallBack(const geometry_msgs::PointStamped::ConstPtr& msg){
    localPoint = msg->point;
    //ROS_INFO("x: %f,y:%f,z:%f", localPoint.x,localPoint.y,localPoint.z);
}

void FlightControl::FlightControlNode::GetGpsHeightCallBack(const sensor_msgs::NavSatFix::ConstPtr & msg){
    HeightGps = msg->altitude;
    //ROS_INFO("HeightGps: %lf",HeightGps);
}

void FlightControl::FlightControlNode::GetVelocityCallBack(const geometry_msgs::Vector3Stamped::ConstPtr & msg){
    this->HorizontalVelocity = msg->vector;
    //ROS_INFO("Vx:%f, Vy:%f, Vz:%f \n", HorizontalVelocity.y,HorizontalVelocity.x,HorizontalVelocity.z);

}

