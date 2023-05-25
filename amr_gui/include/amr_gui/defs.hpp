/**
 * @file defs.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef DEFS_HPP_
#define DEFS_HPP_

#include <QObject>
#include <QString>
#include <geometry_msgs/PointStamped.h>

struct MapPoint
{
    QString name;
    geometry_msgs::PointStamped mapPoint;
};  Q_DECLARE_METATYPE(MapPoint)

#endif // DEFS_HPP_