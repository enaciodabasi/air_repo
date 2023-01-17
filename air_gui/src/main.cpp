
#include "../include/main_window.hpp"

#include <QApplication>
#include <ros/ros.h>

int main(int argc, char** argv)
{

    QApplication app(argc, argv);
    ros::init(argc, argv ,"air_gui");

    ros::NodeHandle nh;

    MainWindow mw(nh, &app);
    mw.show();

    while(ros::ok())
    {
        ros::spinOnce();
        app.exec();
    }

    return 0;
}
