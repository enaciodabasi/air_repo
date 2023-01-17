#include "../include/central_widget.hpp"

CentralWidget::CentralWidget(std::shared_ptr<CommsHandler>& comms_handler, QWidget* parent)
    : QWidget(parent)
{

    m_AxisWidget = new AxisWidget(comms_handler);

    m_StartStopWidget = new StartStopWidget(comms_handler);

    m_ManualControlWidget = new ManualControlWidget(comms_handler);

    m_MainLayout = new QVBoxLayout();

    m_MainLayout->addWidget(m_StartStopWidget, 1);

    m_MainLayout->addWidget(m_AxisWidget, 2);

    m_MainLayout->addWidget(m_ManualControlWidget, 0);

    this->setLayout(m_MainLayout);

}

CentralWidget::~CentralWidget()
{

}