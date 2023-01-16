#ifndef AXIS_WIDGET_HPP
#define AXIS_WIDGET_HPP

#include <QWidget>
#include <QObject>
#include <QSlider>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QLabel>

#include "comms_handler.hpp"

class AxisWidget : public QWidget
{

    Q_OBJECT

    public:

    AxisWidget(std::shared_ptr<CommsHandler>& comms_handler, QWidget* parent = nullptr);
    virtual ~AxisWidget();

    private:

    std::shared_ptr<CommsHandler> m_CommsHandler;

    QVBoxLayout* m_MainLayout;

    QHBoxLayout* m_SliderLayout;

    QSlider* m_LinearVelSlider;
    
    QSlider* m_AngularVelSlider;




};

#endif
