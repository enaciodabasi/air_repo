#include "../include/central_widget.hpp"

CentralWidget::CentralWidget(std::shared_ptr<CommsHandler>& comms_handler, QWidget* parent)
    : QWidget(parent)
{

    m_AxisWidget = new AxisWidget(comms_handler);

    m_StartStopWidget = new StartStopWidget(comms_handler);

    m_ManualControlWidget = new ManualControlWidget(comms_handler);

    m_RvizWidget = new RvizWidget();

    m_ControlsLayout = new QVBoxLayout();
    m_ControlsLayout->addWidget(m_StartStopWidget, 1);
    m_ControlsLayout->addWidget(m_AxisWidget, 2);
    m_ControlsLayout->addWidget(m_ManualControlWidget, 0);

    m_SystemDiagnosticsWidget = new SystemDiagnosticsWidget();

    m_MainLayout = new QHBoxLayout();
    m_MainLayout->addLayout(m_ControlsLayout);
    m_MainLayout->addWidget(m_RvizWidget);
    m_MainLayout->addWidget(m_SystemDiagnosticsWidget);

    this->setLayout(m_MainLayout);

}

CentralWidget::~CentralWidget()
{

}