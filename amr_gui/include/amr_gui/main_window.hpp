/**
 * @file main_window.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef MAIN_WINDOW_HPP_
#define MAIN_WINDOW_HPP_

#include <QMainWindow>
#include <QWidget>
#include <QObject>
#include <QHBoxLayout>

#include <ros/ros.h>
#include <rviz/visualization_frame.h>

#include "amr_gui/robot_info_widget.hpp"
#include "amr_gui/ros_comm.hpp"
#include "amr_gui/map_points_widget.hpp"

class CentralWidget : public QWidget
{
    Q_OBJECT

    public:

    CentralWidget(std::shared_ptr<RosComm> ros_comm, QWidget* parent = nullptr);
    ~CentralWidget();

    private:

    QHBoxLayout* m_MainLayout;

    std::shared_ptr<RosComm> m_RosComm;

    std::shared_ptr<rviz::VisualizationFrame> m_RvizFrame;

    RobotInfoWidget* m_RobotInfoWidget;

    MapPointsWidget* m_MapPointsWidget;

};

class MainWindow : public QMainWindow
{
    Q_OBJECT

    public:

    MainWindow(ros::NodeHandle& nh, QWidget* parent = nullptr);

    ~MainWindow();

    private:

    CentralWidget* m_CentralWidget;

    std::shared_ptr<RosComm> m_RosComm;

};

#endif // MAIN_WINDOW_HPP_
