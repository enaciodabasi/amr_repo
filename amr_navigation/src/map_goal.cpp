/**
 * @file map_goal.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/service_server.h>
#include <amr_custom_interfaces/MapNavigationGoal.h>

bool sendMapNavGoal(amr_custom_interfaces::MapNavigationGoal::Request& request, amr_custom_interfaces::MapNavigationGoal::Response &response)
{
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseClient("move_base", true);

    while(!moveBaseClient.waitForServer(ros::Duration(2.0))){}
    if(!moveBaseClient.isServerConnected())
    {
        return false;
    }
    
    move_base_msgs::MoveBaseGoal goal;
    const auto goalPos = request.target_position;
    goal.target_pose = goalPos;
    
    moveBaseClient.sendGoal(goal);

    bool res = moveBaseClient.waitForResult();

    response.result = res;

    return true;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_goal_server");

    ros::NodeHandle nh;

    ros::ServiceServer mapGoalService = nh.advertiseService("map_goal_server", sendMapNavGoal);

    ros::spin();

    return 0;
}