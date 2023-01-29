#ifndef CENTRAL_WIDGET_HPP
#define CENTRAL_WIDGET_HPP

#include <QWidget>

#include <iostream>
#include <memory>

#include "comms_handler.hpp"
#include "axis_widget.hpp"
#include "start_stop_widget.hpp"
#include "manual_control_widget.hpp"
#include "rviz_widget.hpp"
#include "system_diagnostics_widget.hpp"

class CentralWidget : public QWidget
{
    Q_OBJECT

    public:

    CentralWidget(std::shared_ptr<CommsHandler>& comms_handler, QWidget* parent = nullptr);
    virtual ~CentralWidget();

    private:

    QVBoxLayout* m_ControlsLayout;

    QHBoxLayout* m_MainLayout;

    AxisWidget* m_AxisWidget;

    StartStopWidget* m_StartStopWidget;

    RvizWidget* m_RvizWidget;

    ManualControlWidget* m_ManualControlWidget;

    SystemDiagnosticsWidget* m_SystemDiagnosticsWidget;

};

#endif