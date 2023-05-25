/**
 * @file main_window.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "amr_gui/main_window.hpp"

CentralWidget::CentralWidget(std::shared_ptr<RosComm> ros_comm, QWidget* parent)
    : QWidget(parent)
{

    m_RosComm = ros_comm;

    m_MainLayout = new QHBoxLayout(this);

    m_RvizFrame = std::make_shared<rviz::VisualizationFrame>(new rviz::VisualizationFrame());
    m_RvizFrame->initialize("");

    m_MainLayout->addWidget(m_RvizFrame.get());

    m_MapPointsWidget = new MapPointsWidget();
    m_MainLayout->addWidget(m_MapPointsWidget);
    
    m_RobotInfoWidget = new RobotInfoWidget(ros_comm);
    m_MainLayout->addWidget(m_RobotInfoWidget);

    connect(
        m_RosComm.get(),
        &RosComm::newMapPoint,
        m_MapPointsWidget,
        &MapPointsWidget::onNewMapPoint
    );

    connect(
        m_MapPointsWidget,
        &MapPointsWidget::newNavTarget,
        m_RosComm.get(),
        &RosComm::onNewNavTarget
    );

    this->setLayout(m_MainLayout);
}

CentralWidget::~CentralWidget()
{

}

MainWindow::MainWindow(ros::NodeHandle& nh, QWidget* parent)
    : QMainWindow(parent)
{

    m_RosComm = std::make_shared<RosComm>(nh);

    m_CentralWidget = new CentralWidget(m_RosComm, this);    

    this->setCentralWidget(m_CentralWidget);

}

MainWindow::~MainWindow()
{

}