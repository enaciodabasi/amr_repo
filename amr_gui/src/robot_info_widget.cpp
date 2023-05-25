/**
 * @file robot_info_widget.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "amr_gui/robot_info_widget.hpp"

JointInfoWidget::JointInfoWidget(const QString& joint_name, QWidget* parent)
    : QWidget(parent)
{
    m_MainLayout = new QVBoxLayout();

    m_JointNameLabel = new QLabel(joint_name);
    m_MainLayout->addWidget(m_JointNameLabel, 20, Qt::AlignVCenter);

    m_InfoDisplayLayout = new QFormLayout();
    
    m_JointPositionLabel = new QLabel();
    m_InfoDisplayLayout->addRow("Joint Position: ", m_JointPositionLabel);
    
    m_JointVelocityLabel = new QLabel();
    m_InfoDisplayLayout->addRow("Joint Velocity: ", m_JointVelocityLabel);

    m_MainLayout->addLayout(m_InfoDisplayLayout, 80);
    this->setLayout(m_MainLayout);

}

JointInfoWidget::~JointInfoWidget()
{

}

RobotInfoWidget::RobotInfoWidget(std::shared_ptr<RosComm> ros_comm_shared_ptr, QWidget* parent)
    : QWidget(parent)
{
    m_RosComm = ros_comm_shared_ptr;
    
    m_UpdateTimer = new QTimer(this);
    m_UpdateTimer->setInterval(std::chrono::milliseconds(500));

    m_LeftWheelInfoWidget = new JointInfoWidget("Left Wheel", this);
    m_RightWheelInfoWidget = new JointInfoWidget("Right Wheel", this);

    m_MainLayout = new QVBoxLayout();

    m_JointInfoDisplaysLayout = new QHBoxLayout();
    m_JointInfoDisplaysLayout->addWidget(m_LeftWheelInfoWidget);
    m_JointInfoDisplaysLayout->addWidget(m_RightWheelInfoWidget);

    m_MainLayout->addLayout(m_JointInfoDisplaysLayout);
    this->setLayout(m_MainLayout);

    connect(
        m_UpdateTimer,
        &QTimer::timeout,
        this,
        QOverload<>::of(&RobotInfoWidget::update)
    );

    m_UpdateTimer->start();

}

RobotInfoWidget::~RobotInfoWidget()
{

}

void RobotInfoWidget::update()
{
    const auto currJointInfo = m_RosComm->getJointInfo();
    
    std::pair<double, double> leftWheelInfo(0.0, 0.0);
    std::pair<double, double> rightWheelInfo(0.0, 0.0);

    for(std::size_t i = 0; i < currJointInfo.name.size(); i++)
    {
        if(currJointInfo.name.at(i) == m_LeftWheelJointName)
        {
            leftWheelInfo.first = currJointInfo.position[i];
            leftWheelInfo.second = currJointInfo.velocity[i];
        }
        else if(currJointInfo.name.at(i) == m_RightWheelJointName)
        {
            rightWheelInfo.first = currJointInfo.position[i];
            rightWheelInfo.second = currJointInfo.velocity[i];
        }
        else
            continue;
    }

    m_LeftWheelInfoWidget->update(QString::number(leftWheelInfo.first), QString::number(leftWheelInfo.second));
    m_RightWheelInfoWidget->update(QString::number(rightWheelInfo.first), QString::number(rightWheelInfo.second));

}