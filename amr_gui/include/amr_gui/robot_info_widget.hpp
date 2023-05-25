/**
 * @file robot_info_widget.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef ROBOT_INFO_WIDGET_HPP_
#define ROBOT_INFO_WIDGET_HPP_

#include <QWidget>
#include <QObject>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QLabel>
#include <QTimer>

#include "ros_comm.hpp"

class JointInfoWidget : public QWidget
{
    public:

    JointInfoWidget(const QString& joint_name, QWidget* parent = nullptr);

    ~JointInfoWidget();

    inline void update(
        const QString& new_joint_pos,
        const QString& new_joint_vel
    )
    {
        m_JointPositionLabel->setText(new_joint_pos);
        m_JointVelocityLabel->setText(new_joint_vel);
    }

    private:

    QVBoxLayout* m_MainLayout;

    QFormLayout* m_InfoDisplayLayout;

    QLabel* m_JointNameLabel;

    QLabel* m_JointPositionLabel;

    QLabel* m_JointVelocityLabel;

};

class RobotInfoWidget : public QWidget
{
    public:

    RobotInfoWidget(std::shared_ptr<RosComm> ros_comm_shared_ptr, QWidget* parent = nullptr);

    ~RobotInfoWidget();

    private:

    QVBoxLayout* m_MainLayout;
    QHBoxLayout* m_JointInfoDisplaysLayout;

    std::shared_ptr<RosComm> m_RosComm;
    QTimer* m_UpdateTimer;

    JointInfoWidget* m_LeftWheelInfoWidget;
    JointInfoWidget* m_RightWheelInfoWidget;

    std::string m_LeftWheelJointName;
    std::string m_RightWheelJointName;

    void update();

};

#endif // ROBOT_INFO_WIDGET_HPP_