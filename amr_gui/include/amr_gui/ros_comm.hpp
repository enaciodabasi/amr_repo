/**
 * @file ros_comm.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef ROS_COMM_HPP_
#define ROS_COMM_HPP_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/PoseStamped.h>  
#include <geometry_msgs/PointStamped.h>    
#include <ros/service_client.h>

#include <amr_custom_interfaces/MapNavigationGoal.h>

#include <queue>
#include <optional>
#include <thread>

#include <QObject>

#include "defs.hpp"

class RosComm : public QObject
{

    Q_OBJECT

    public:

    RosComm(ros::NodeHandle& nh);

    ~RosComm();

    const sensor_msgs::JointState getJointInfo();

    const sensor_msgs::BatteryState getBatteryStateInfo();

    public slots:

    void onNewNavTarget(const MapPoint& map_point);

    signals:

    void newMapPoint(const MapPoint& map_point);

    private:

    ros::NodeHandle m_Nh;

    ros::Subscriber m_JointInfoSub;
    
    std::queue<sensor_msgs::JointState> m_JointStateQueue;

    ros::Subscriber m_BatteryStateSub;

    std::queue<sensor_msgs::BatteryState> m_BatteryStateInfoQueue;

    ros::Subscriber m_MapPointSub;

    ros::ServiceClient m_MapGoalClient;
    
    std::string m_JointInfoTopicName = "/joint_states";

    std::string m_BatteryStateTopicName = "/amr/battery_state";

    void jointInfoCallback(const sensor_msgs::JointState& joint_info);

    void batteryStateCallback(const sensor_msgs::BatteryState& battery_info);

    void mapPointCallback(const geometry_msgs::PointStamped& map_point);

};

#endif // ROS_COMM_HPP_