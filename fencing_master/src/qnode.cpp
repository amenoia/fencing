/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/fencing_master/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

ros::Publisher fencing_master_to_Pub;
ros::Subscriber fencing_vision_to_Sub;
ros::Subscriber fencing_ui_to_Sub;


ros::Publisher motionNum_Pub;
ros::Subscriber imu_Sub;
ros::Subscriber motion_end_Sub;

namespace fencing_master {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{}

QNode::~QNode() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"fencing_master");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.

    fencing_master_to_Pub = n.advertise<msg_generate::fencing_master_to>("fencing_master_to", 100);
    fencing_vision_to_Sub = n.subscribe("fencing_vision_to", 100, &QNode::vision_callback, this);
    fencing_ui_to_Sub = n.subscribe("fencing_ui_to", 100, &QNode::ui_callback, this);

    motionNum_Pub = n.advertise<msg_generate::motionNum_msg>("Motion", 100);
    imu_Sub = n.subscribe("imu", 100, &QNode::imu_callback, this);
    motion_end_Sub = n.subscribe("motion_end", 100, &QNode::motion_end_callback, this);

    start();
    return true;
}


void QNode::run() {
    ros::Rate loop_rate(33);
    while ( ros::ok() ) {

        ros::spinOnce();
        loop_rate.sleep();
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


}  // namespace fencing_master
