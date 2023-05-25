/**
 * @file hardware_interface.cpp
 * @author Eren Naci Odabasi (enaciodabasi@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-12
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "../include/amr_hardware_interface.hpp"

namespace amr
{
    HardwareInterface::HardwareInterface(ros::NodeHandle& nh, bool enable_homing)
        : m_EnableHoming(enable_homing)
    {

        //m_ControllerManagerSharedPtr = std::make_shared<controller_manager::ControllerManager>(this, m_Node);

        m_LeftJointHandle = std::make_shared<WheelJointHandle>();
        m_RightJointHandle = std::make_shared<WheelJointHandle>();

        configure();

        m_RtVelCmdPub = std::make_shared<realtime_tools::RealtimePublisher<amr_custom_interfaces::WheelVelCmd>>(m_Node, wheelVelCmdPubTopicName, 100);

        m_EncoderDataSub = m_Node.subscribe(
            encoderSubTopicName,
            1,
            &HardwareInterface::encoderDataCallback,
            this
        );

        hardware_interface::JointStateHandle leftWheelHandle(
            m_LeftJointHandle->name,
            &m_LeftJointHandle->position,
            &m_LeftJointHandle->velocity,
            &m_LeftJointHandle->effort
        );

        m_JointStateInterface.registerHandle(leftWheelHandle);
        hardware_interface::JointStateHandle rightWheelHandle(
            m_RightJointHandle->name,
            &m_RightJointHandle->position,
            &m_RightJointHandle->velocity,
            &m_RightJointHandle->effort
        );
        m_JointStateInterface.registerHandle(rightWheelHandle);

        hardware_interface::JointHandle leftWheelVelHandle(
            leftWheelHandle,
            &m_LeftJointHandle->vel_command
        );
        m_VelJointInterface.registerHandle(leftWheelVelHandle);

        hardware_interface::JointHandle rightWheelVelHandle(
            rightWheelHandle,
            &m_RightJointHandle->vel_command
        );
        m_VelJointInterface.registerHandle(rightWheelVelHandle);

        this->registerInterface(&m_JointStateInterface);
        this->registerInterface(&m_VelJointInterface);

    }

    HardwareInterface::~HardwareInterface()
    {

    }

    void HardwareInterface::read(const ros::Time& time, const ros::Duration& period)
    {

        const auto encoderCounts = *(m_EncoderDataBuffer.readFromRT());
        // todo: encoder to position

        double leftWheelJointPosition;
        double rightWheelJointPosition;

        

        if(m_EnableHoming)
        {
            if(m_HomingHelper.initialRead)
            {
                m_HomingHelper.leftPosDiff = leftWheelJointPosition;
                m_HomingHelper.rightPosDiff = rightWheelJointPosition;
                m_HomingHelper.initialRead = false;
            }
        }


        if(m_EnableHoming)
        {
            leftWheelJointPosition -= m_HomingHelper.leftPosDiff;
            rightWheelJointPosition -= m_HomingHelper.rightPosDiff;
        }

        m_LeftJointHandle->position = leftWheelJointPosition;
        m_RightJointHandle->position = rightWheelJointPosition;

    }

    void HardwareInterface::write(const ros::Time& time, const ros::Duration& period)
    {
        
        // Convert to driver-language

        if(m_RtVelCmdPub->trylock())
        {
            m_RtVelCmdPub->msg_.timestamp = time;
            m_RtVelCmdPub->msg_.leftWHeelVelCmd;
            m_RtVelCmdPub->msg_.rightWheelVelCmd;
        }
    }

    void HardwareInterface::configure()
    {

    }

    void HardwareInterface::encoderDataCallback(const amr_custom_interfaces::EncoderData& encoder_data)
    {
        m_EncoderDataBuffer.writeFromNonRT(encoder_data);
    }

    double HardwareInterface::motorPositionToWheelPosition(const int64_t encoder_count)
    {
        double motorPosition = 
            (double)encoder_count / 
            (encodingMultiplier * m_MotorParams.encoderResolution);

        return motorPosition * m_MotorParams.gearRatio;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "amr_hardware_interface_node");

    ros::NodeHandle nh;
    amr::HardwareInterface amr_hw(nh);

    controller_manager::ControllerManager cm(&amr_hw, nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::Time prevTime = ros::Time::now();
    ros::Rate sleepRate(1.0/amr_hw.getHwFrequency());
    sleepRate.sleep();

    while(ros::ok())
    {
        const ros::Time currentTime = ros::Time::now();
        const ros::Duration period = currentTime - prevTime;
        prevTime = currentTime;

        amr_hw.read(currentTime, period);

        cm.update(currentTime, period);

        amr_hw.write(currentTime, period);

        sleepRate.sleep();

    }   

    ros::waitForShutdown();

    return 0;
}