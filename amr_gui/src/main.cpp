/**
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <QApplication>

#include <ros/ros.h>

#include "amr_gui/main_window.hpp"
#include "amr_gui/defs.hpp"


int main(int argc, char** argv)
{
    qRegisterMetaType<MapPoint>();

    ros::init(argc, argv, "amr_gui_node");
    ros::NodeHandle nh;
    QApplication amrApp(argc, argv);

    MainWindow* mainWindow = new MainWindow(nh);
    mainWindow->show();

    while(ros::ok())
    {
        ros::spinOnce();
        amrApp.exec();    
    }

    amrApp.exit();

    return 0;
}