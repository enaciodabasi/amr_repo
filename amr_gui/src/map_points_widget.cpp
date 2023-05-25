/**
 * @file map_points_widget.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "amr_gui/map_points_widget.hpp"

MapPointsWidget::MapPointsWidget(QWidget* parent)
    : QWidget(parent)
{
    m_MainLayout = new QVBoxLayout(this);

    m_WidgetNameLabel = new QLabel("Map Points");
    m_MainLayout->addWidget(m_WidgetNameLabel, 5, Qt::AlignVCenter);

    m_GoalLayout = new QHBoxLayout();

    m_TargetPointName = new QLineEdit();
    m_TargetPointName->clear();

    m_GoToPointButton = new QPushButton("Go");
    
    m_GoalLayout = new QHBoxLayout();
    m_GoalLayout->addWidget(m_TargetPointName, 60, Qt::AlignLeft);
    m_GoalLayout->addWidget(m_GoToPointButton, 40, Qt::AlignRight);
    m_GoalLayout->setAlignment(Qt::AlignBottom);
    
    m_MainLayout->addLayout(m_GoalLayout, 10);



    connect(
        m_GoToPointButton,
        &QPushButton::clicked,
        this,
        &MapPointsWidget::onGoToPointBtnClicked
    );

    this->setLayout(m_MainLayout);
}

MapPointsWidget::~MapPointsWidget()
{
    for(auto mpd : m_MapPoints)
        delete mpd;
}

void MapPointsWidget::onGoToPointBtnClicked()
{
    if(m_TargetPointName->text().isEmpty())
        return;
    
    const QString targetPointName = m_TargetPointName->text();

    auto found = std::find_if(
        m_MapPoints.begin(),
        m_MapPoints.end(),
        [&targetPointName](MapPointDisplay* mpd){
            return mpd->m_MapPoint->name == targetPointName;
        }
    );

    if(found == m_MapPoints.end())
        return;

    const auto targetPointCoord = *((*found)->m_MapPoint);

    emit newNavTarget(targetPointCoord);
}

void MapPointsWidget::onNewMapPoint(const MapPoint& map_point)
{
    bool ok;
    QString pointName = QInputDialog::getText(
        this,
        tr("New Map Point Name"),
        tr("Name:"),
        QLineEdit::Normal,
        QDir::home().dirName(),
        &ok
    );

    auto point = map_point;
    if(ok && !pointName.isEmpty())
    {
        point.name = pointName;
    }

    MapPointDisplay* mpd = new MapPointDisplay(this);
    mpd->init(point);

    m_MapPoints.push_back(mpd);
    m_MainLayout->addWidget(mpd);

}

MapPointDisplay::MapPointDisplay(QWidget *parent)
{
    m_MainLayout = new QVBoxLayout();

    //m_PointInfoLayout = new QHBoxLayout();

    m_PointNameLabel = new QLabel();
    m_MainLayout->addWidget(m_PointNameLabel, 0, Qt::AlignVCenter);

    m_PositionInfoLayout = new QFormLayout();
    m_PositionInfo.x = new QLabel();
    m_PositionInfo.y = new QLabel();
    m_PositionInfo.z = new QLabel();

    m_PositionInfoLayout->addWidget(new QLabel("Position"));
    m_PositionInfoLayout->addRow("x: ", m_PositionInfo.x);
    m_PositionInfoLayout->addRow("y: ", m_PositionInfo.y);
    m_PositionInfoLayout->addRow("z: ", m_PositionInfo.z);

    m_MainLayout->addLayout(m_PositionInfoLayout);

    this->setLayout(m_MainLayout);
}

MapPointDisplay::~MapPointDisplay()
{

}

void MapPointDisplay::init(const MapPoint& map_point)
{
    m_MapPoint = std::make_unique<MapPoint>(std::move(map_point));
    m_PointNameLabel->setText(map_point.name);
    m_PositionInfo.x->setText(QString::number(m_MapPoint->mapPoint.point.x, (char)103, 8));
    m_PositionInfo.y->setText(QString::number(m_MapPoint->mapPoint.point.y, (char)103, 8));
    m_PositionInfo.z->setText(QString::number(m_MapPoint->mapPoint.point.z, (char)103, 8));
}