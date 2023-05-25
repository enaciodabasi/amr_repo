/**
 * @file hardware_interface.hpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef AMR_HARDWARE_INTERFACE_HPP_
#define AMR_HARDWARE_INTERFACE_HPP_

#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>

#include <amr_custom_interfaces/EncoderData.h>
#include <amr_custom_interfaces/WheelVelCmd.h>

#include <iostream>
#include <vector>
#include <string>
#include <memory>

namespace amr
{

    enum encodingMethod : int
    {
        X1=1, 
        X2=2, 
        X3=4};

    struct WheelJointHandle
    {
        WheelJointHandle()
        {
            name = "";
            position = 0.0;
            velocity = 0.0;
            effort = 0.0;
            vel_command = 0.0;
        }

        std::string name;
        double position;
        double velocity;
        double effort;
        double vel_command;
    };

    class HardwareInterface : public hardware_interface::RobotHW
    {
        public:

        HardwareInterface(ros::NodeHandle& nh, bool enable_homing=true);

        ~HardwareInterface();

        void read(const ros::Time& time, const ros::Duration& period);

        void write(const ros::Time& time, const ros::Duration& period);

        const double getHwFrequency() const
        {
            return m_LoopFrequency;
        }

        private:

        ros::NodeHandle m_Node;

        //std::shared_ptr<controller_manager::ControllerManager> m_ControllerManagerSharedPtr;
        
        ros::Subscriber m_EncoderDataSub;
        realtime_tools::RealtimeBuffer<amr_custom_interfaces::EncoderData> m_EncoderDataBuffer;

        std::shared_ptr<realtime_tools::RealtimePublisher<amr_custom_interfaces::WheelVelCmd>> m_RtVelCmdPub;
        //realtime_tools::RealtimeBuffer<amr_custom_interfaces::WheelVelCmd> m_Commands;

        std::shared_ptr<WheelJointHandle> m_LeftJointHandle;

        std::shared_ptr<WheelJointHandle> m_RightJointHandle;

        std::size_t m_NumJoints;

        hardware_interface::JointStateInterface m_JointStateInterface;

        hardware_interface::VelocityJointInterface m_VelJointInterface;

        double m_LoopFrequency;

        std::string encoderSubTopicName;
        
        std::string wheelVelCmdPubTopicName;

        bool m_EnableHoming = true;
        struct
        {       
            bool initialRead = true;
            double leftPosDiff = 0.0;
            double rightPosDiff = 0.0;
        }m_HomingHelper;

        struct
        {

        }m_WheelParams;

        struct 
        {
            double encoderResolution;

            double gearRatio;

        }m_MotorParams;

        int encodingMultiplier;
        
        void configure();

        void encoderDataCallback(const amr_custom_interfaces::EncoderData& encoder_data);

        double motorPositionToWheelPosition(const int64_t encoder_count);

    };
}

#endif // AMR_HARDWARE_INTERFACE_HPP_