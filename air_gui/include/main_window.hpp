#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <memory>
#include <iostream>

#include <QMainWindow>

#include <ros/ros.h>

#include "comms_handler.hpp"
#include "central_widget.hpp"


class MainWindow : public QMainWindow
{

    Q_OBJECT

    public:

    MainWindow(ros::NodeHandle& nh, QApplication* app, QWidget* parent = nullptr);
    virtual ~MainWindow();

    private:

    std::shared_ptr<CommsHandler> m_CommsHandler;

    CentralWidget* m_CentralWidget;

};

#endif