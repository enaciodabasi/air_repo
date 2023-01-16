#ifndef CENTRAL_WIDGET_HPP
#define CENTRAL_WIDGET_HPP

#include <QWidget>

#include <iostream>
#include <memory>

#include "comms_handler.hpp"
#include "axis_widget.hpp"

class CentralWidget : public QWidget
{
    Q_OBJECT

    public:

    CentralWidget(std::shared_ptr<CommsHandler>& comms_handler, QWidget* parent = nullptr);
    virtual ~CentralWidget();

    private:

    QHBoxLayout* m_MainLayout;

    AxisWidget* m_AxisWidget;


};

#endif