#ifndef RVIZ_WIDGET_HPP
#define RVIZ_WIDGET_HPP

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"

#include <QWidget>
#include <QVBoxLayout>

class RvizWidget: public QWidget
{
    Q_OBJECT

    public:

    RvizWidget(QWidget* parent = nullptr);
    virtual ~RvizWidget();

    private:

    QVBoxLayout* m_MainLayout;

    rviz::VisualizationManager* m_VisManager;

    rviz::RenderPanel* m_RenderPanel;

    rviz::Display* m_Grid;

    rviz::Display* m_RobotDesc;

    rviz::Display* m_LaserScan;

    rviz::Display* m_Map;

};


#endif