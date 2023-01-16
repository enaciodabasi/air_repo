#include "../include/axis_widget.hpp"

AxisWidget::AxisWidget(std::shared_ptr<CommsHandler>& comms_handler, QWidget* parent)
    : QWidget(parent)
{
    m_CommsHandler = comms_handler;

    m_MainLayout = new QVBoxLayout();

    m_SliderLayout = new QHBoxLayout();

    m_LinearVelSlider = new QSlider(Qt::Vertical);
    
    m_AngularVelSlider = new QSlider(Qt::Horizontal);

    m_SliderLayout->addWidget(new QLabel("Linear Velocity:"));
    m_SliderLayout->addWidget(m_LinearVelSlider);

    m_SliderLayout->addWidget(new QLabel("Angular Velocity:"));
    m_SliderLayout->addWidget(m_AngularVelSlider);

    m_MainLayout->addLayout(m_SliderLayout);

    this->setLayout(m_MainLayout);

}

AxisWidget::~AxisWidget()
{

}