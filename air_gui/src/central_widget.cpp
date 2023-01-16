#include "../include/central_widget.hpp"

CentralWidget::CentralWidget(std::shared_ptr<CommsHandler>& comms_handler, QWidget* parent)
    : QWidget(parent)
{

    m_AxisWidget = new AxisWidget(comms_handler, this);

    m_MainLayout = new QHBoxLayout();

    m_MainLayout->addWidget(m_AxisWidget);

}

CentralWidget::~CentralWidget()
{

}