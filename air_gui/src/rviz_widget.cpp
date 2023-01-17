#include "../include/rviz_widget.hpp"

RvizWidget::RvizWidget(QWidget* parent)
    : QWidget(parent)
{

    m_MainLayout = new QVBoxLayout();

    m_RenderPanel = new rviz::RenderPanel();
    m_MainLayout->addWidget(m_RenderPanel);

    m_VisManager = new rviz::VisualizationManager(m_RenderPanel);
    m_RenderPanel->initialize(m_VisManager->getSceneManager(), m_VisManager);
    m_VisManager->initialize();
    m_VisManager->startUpdate();

    m_Grid = m_VisManager->createDisplay("rviz/Grid", "Grid", true);

    m_RobotDesc = m_VisManager->createDisplay("rviz/RobotModel", "Robot Model", true);

    m_LaserScan = m_VisManager->createDisplay("rviz/LaserScan", "Laser Scan", true);
    m_LaserScan->subProp("Topic")->setValue("/scan");

    m_Map = m_VisManager->createDisplay("rviz/Map", "Map" ,true);
    m_Map->subProp("Topic")->setValue("/map");

    m_VisManager->setFixedFrame("map");

    this->setLayout(m_MainLayout);
}

RvizWidget::~RvizWidget()
{
    if(m_VisManager)
    {
        m_VisManager->stopUpdate();
        m_VisManager->deleteLater();
    }


}