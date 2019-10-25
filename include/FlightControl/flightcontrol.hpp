#ifndef FLIGHTCONTROL_H
#define FLIGHTCONTROL_H

//ros include
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/UInt8.h>

//DJI include 
#include "../../../../devel/include/dji_sdk/DroneTaskControl.h"
#include "../../../../devel/include/dji_sdk/SDKControlAuthority.h"

#include "../../../dji_sdk/include/dji_sdk/dji_sdk.h"

#include <tf/tf.h>
#include <sensor_msgs/Joy.h>
#include "FlightControl/state.h"
#include "FlightControl/opticalflow.h"

namespace FlightControl{

class FlightControlNode{

public:
    FlightControlNode(ros::NodeHandle &n);
    void InitSubcribers(ros::NodeHandle &n);
    void InitPublishers(ros::NodeHandle &n);
    void InitServices(ros::NodeHandle &n);
    void InitFlightControlThread();
    void FlightControlThread();   //run flight control algorithm 
    
    
    //void Publish();

    bool ObtainControl();
    bool MonitoredTakeoff();
    bool TakeoffLand(int task);

private:
    ros::Subscriber AttitudeSubscriber;     //Quaternion
    ros::Subscriber FlightStatusSubscriber; //uint8_t
    ros::Subscriber DisplayModeSubscriber;  //Details can be found in DisplayMode enum in dji_sdk.h
    ros::Subscriber HeightSubscriber;
    ros::Subscriber GpsHeightSubscriber;
    ros::Subscriber HorizontalVelocitySubscriber;
    //ros::Subscriber DeltaPositionSubscriber;
   
    //Guidance
    ros::Subscriber GuidanceUltrasonicSubscriber;
    ros::Subscriber GuidanceVelocitySubscriber;
    ros::Subscriber GuidanceObstacleDistanceSubscriber;
    //ros::Subscriber GuidanceIMUSubscriber;
    
    ros::Publisher CtrAttitudePublisher;
   
    ros::ServiceClient CtrlAuthorityService;
    ros::ServiceClient DroneTaskControlService;

    void GetQuaternionCallBack(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
    void GetFlightStatusCallBack(const std_msgs::UInt8::ConstPtr& msg);
    void GetDisplayModeCallBack(const std_msgs::UInt8::ConstPtr& msg);
    void GetVelocityCallBack(const geometry_msgs::Vector3Stamped::ConstPtr & msg);
    void GetGpsHeightCallBack(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void GetHeightCallBack(const FlightControl::state::ConstPtr& msg);
    //void GetDeltaPositionCallBack(const FlightControl::opticalflow::ConstPtr& msg);

    void GetHeightGuidanceCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
    void GetVelocityGuidanceCallBack(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
    void GetObstacleDistanceCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
    //void GetImuGuidanceCallBack(const geometry_msgs::TransformStamped::ConstPtr &msg);

        
    geometry_msgs::Vector3 HorizontalVelocity;  //from imu of drone
    geometry_msgs::Quaternion Attitude;         //from imu of drone 
    uint8_t FlightStatus;
    uint8_t DisplayMode;

    //The height of UAV
    double Height;  //Height from guidance
    double HeightGps; //Height from Gps
    double HeightAboveTakeoff;

    //Velocity from guidance
    float Vx;
    float Vy;
    float Vz;

    float ObstacleDistance; 
};

}

#endif
