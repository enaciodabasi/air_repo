#include "../include/main_window.hpp"

MainWindow::MainWindow(ros::NodeHandle& nh, QApplication* app, QWidget* parent)
    : QMainWindow(parent)
{

    m_CommsHandler = std::make_shared<CommsHandler>(nh);

    m_CentralWidget = new CentralWidget(m_CommsHandler);

    this->setCentralWidget(m_CentralWidget);

}

MainWindow::~MainWindow()
{

}