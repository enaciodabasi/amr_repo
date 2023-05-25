/**
 * @file ros_comm.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "amr_gui/ros_comm.hpp"

#include <QDebug>

RosComm::RosComm(ros::NodeHandle& nh)
    : QObject(), m_Nh(nh)
{

    m_JointStateQueue.push(sensor_msgs::JointState());

    m_JointInfoSub = m_Nh.subscribe(
        m_JointInfoTopicName,
        10,
        &RosComm::jointInfoCallback,
        this
    );

    m_BatteryStateSub = m_Nh.subscribe(
        m_BatteryStateTopicName,
        10,
        &RosComm::batteryStateCallback,
        this
    );
    
    m_MapPointSub = m_Nh.subscribe(
        "/clicked_point",
        1,
        &RosComm::mapPointCallback,
        this
    );

    

}

RosComm::~RosComm()
{

}

void RosComm::onNewNavTarget(const MapPoint& map_point)
{
    
    std::thread goalClientThread(
        [&map_point](){
            ros::NodeHandle nh;

            ros::ServiceClient goalClient = nh.serviceClient<amr_custom_interfaces::MapNavigationGoal>(
                "map_goal_server"
            );

            amr_custom_interfaces::MapNavigationGoal goal;
            goal.request.target_position.header.frame_id = "map";
            goal.request.target_position.header.stamp = ros::Time::now();
            goal.request.target_position.pose.position.x = map_point.mapPoint.point.x;
            goal.request.target_position.pose.position.y = map_point.mapPoint.point.y;
            goal.request.target_position.pose.position.z = map_point.mapPoint.point.z;
            goal.request.target_position.pose.orientation.w = 1.0;

            if(goalClient.call(goal))
            {

            }
        }
    );

    goalClientThread.detach();
}

const sensor_msgs::JointState RosComm::getJointInfo()
{
    if(m_JointStateQueue.empty())
    {
        return sensor_msgs::JointState();
    }

    const auto currentJointInfo = m_JointStateQueue.front();
    m_JointStateQueue.pop();

    return currentJointInfo;
}

const sensor_msgs::BatteryState RosComm::getBatteryStateInfo()
{
    if(m_BatteryStateInfoQueue.empty())
    {
        return sensor_msgs::BatteryState();
    }
    const auto batteryStateInfo = m_BatteryStateInfoQueue.front();
    m_BatteryStateInfoQueue.pop();

    return batteryStateInfo;
}

void RosComm::jointInfoCallback(const sensor_msgs::JointState& joint_info)
{
    m_JointStateQueue.push(joint_info);
}

void RosComm::batteryStateCallback(const sensor_msgs::BatteryState& battery_state)
{
    m_BatteryStateInfoQueue.push(battery_state);
}

void RosComm::mapPointCallback(const geometry_msgs::PointStamped& map_point)
{
    MapPoint mapPointToRegister;
    mapPointToRegister.name = "Map Point";
    mapPointToRegister.mapPoint = map_point;

    emit newMapPoint(mapPointToRegister);
}