/**
 * @file map_points_widget.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef MAP_POINTS_WIDGET_HPP_
#define MAP_POINTS_WIDGET_HPP_

#include <QWidget>
#include <QObject>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QLabel>
#include <QInputDialog>
#include <QDir>
#include <QLineEdit>
#include <QVector>
#include <QPushButton>

#include <memory>

#include "defs.hpp"

class MapPointDisplay : public QWidget
{
    public:

    MapPointDisplay(QWidget* parent = nullptr);

    ~MapPointDisplay();

    void init(const MapPoint& map_point);

    std::unique_ptr<MapPoint> m_MapPoint;

    private:

    QVBoxLayout* m_MainLayout;

    QHBoxLayout* m_PointInfoLayout;

    QFormLayout* m_PositionInfoLayout;
    
    QFormLayout* m_OrientationInfoLayout;

    QLabel* m_PointNameLabel;

    struct
    {
        QLabel* x;
        QLabel* y;
        QLabel* z;
    } m_PositionInfo;


};

class MapPointsWidget : public QWidget
{
    Q_OBJECT
    
    public:

    MapPointsWidget(QWidget* parent = nullptr);

    ~MapPointsWidget();

    private:

    QVBoxLayout* m_MainLayout;
    
    QLabel* m_WidgetNameLabel;

    QHBoxLayout* m_GoalLayout;

    QLineEdit* m_TargetPointName;
    QPushButton* m_GoToPointButton;

    QVector<MapPointDisplay*> m_MapPoints;

    public slots:

    void onNewMapPoint(const MapPoint& map_point);

    void onGoToPointBtnClicked();

    signals:

    void newNavTarget(const MapPoint& map_point);

};

#endif // MAP_POINTS_WIDGET_HPP_