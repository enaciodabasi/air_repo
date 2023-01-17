#include "../include/axis_widget.hpp"

#include <QDebug>

AxisWidget::AxisWidget(std::shared_ptr<CommsHandler>& comms_handler, QWidget* parent)
    : QWidget(parent)
{
    m_CommsHandler = comms_handler;

    m_MainLayout = new QVBoxLayout();
    m_SliderLayout = new QHBoxLayout();

    m_LinearVelSlider = new QSlider(Qt::Horizontal);
    m_LinearVelSlider->setRange(-1000, 1000);
    m_LinearVelSlider->setValue(0);
    m_AngularVelSlider = new QSlider(Qt::Horizontal);
    m_AngularVelSlider->setRange(-1000, 1000);
    m_AngularVelSlider->setValue(0);

    m_LinearVelValue = new QLineEdit();
    m_LinearVelValue->setReadOnly(true);
    m_AngularVelValue = new QLineEdit();
    m_AngularVelValue->setReadOnly(true);

    QFormLayout* linVelForm = new QFormLayout();
    linVelForm->addRow(new QLabel("Linear Velocity [m/s]:"), m_LinearVelValue);
    QVBoxLayout* linVelLayout = new QVBoxLayout();

    linVelLayout->addLayout(linVelForm);
    linVelLayout->addWidget(m_LinearVelSlider);

    QFormLayout* angVelForm = new QFormLayout();
    angVelForm->addRow(new QLabel("Angular Velocity [rad/s]: "), m_AngularVelValue);
    QVBoxLayout* angVelLayout = new QVBoxLayout();
    //m_SliderLayout->addWidget(new QLabel("Angular Velocity: rad[s]"));
    angVelLayout->addLayout(angVelForm);
    angVelLayout->addWidget(m_AngularVelSlider);

    m_SliderLayout->addLayout(linVelLayout);
    m_SliderLayout->addLayout(angVelLayout);

    m_MainLayout->addLayout(m_SliderLayout);

    this->setLayout(m_MainLayout);

    connect(
        m_LinearVelSlider,
        &QSlider::valueChanged,
        this,
        &AxisWidget::onLinVelSliderChanged
    );

    connect(
        m_AngularVelSlider,
        &QSlider::valueChanged,
        this,
        &AxisWidget::onAngVelSliderChanged
    );

    connect(
        m_LinearVelSlider,
        &QSlider::sliderReleased,
        this,
        &AxisWidget::onVelSliderReleased
    );

    connect(
        m_AngularVelSlider,
        &QSlider::sliderReleased,
        this,
        &AxisWidget::onVelSliderReleased
    );

}

AxisWidget::~AxisWidget()
{

}

void AxisWidget::onLinVelSliderChanged()
{
    //qDebug() << m_LinearVelSlider->value();
    m_LinearVelValue->setText(QString::fromStdString(std::to_string((double)(m_LinearVelSlider->value() / m_VelScalingFactor))));
}

void AxisWidget::onAngVelSliderChanged()
{
    m_AngularVelValue->setText(QString::fromStdString(std::to_string((double)(m_AngularVelSlider->value() / m_VelScalingFactor))));
}

void AxisWidget::onVelSliderReleased()
{
    QObject* s = sender();

    if(s == m_LinearVelSlider)
    {
        m_LinearVelSlider->setValue(0);
    }
    else if(s == m_AngularVelSlider)
    {
        m_AngularVelSlider->setValue(0);
    }
    
}