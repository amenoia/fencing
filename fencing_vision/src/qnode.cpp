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
#include "../include/fencing_vision/qnode.hpp"


/*****************************************************************************
** Namespaces
*****************************************************************************/
ros::Publisher fencing_vision_to_Pub;
using namespace std;

namespace fencing_vision {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{

    img_qnode = NULL;
    infoColor = NULL;



}

QNode::~QNode() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"fencing_vision");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.

    fencing_vision_to_Pub = n.advertise<msg_generate::fencing_vision_to>("fencing_vision_to", 100);

    image_transport::ImageTransport img(n);
    image_encodings_Sub = img.subscribeCamera("/usb_cam/image_raw", 100, &fencing_vision::QNode::imageCallBack, this);

    start();
    return true;
}
void QNode::imageCallBack(const sensor_msgs::ImageConstPtr& msg_img, const sensor_msgs::CameraInfoConstPtr& info_msg)
{


    if(img_qnode == NULL && !isRecv)
    {


        img_qnode = new cv::Mat(cv_bridge::toCvCopy(msg_img,sensor_msgs::image_encodings::BGR8)->image);
        infoColor = new cv::Mat(3,3,CV_64FC1,(void*)info_msg->K.data());
        if(img_qnode != NULL)
        {
            isRecv = true;
            Q_EMIT recvImg(*img_qnode,*infoColor);

            delete img_qnode; delete infoColor;
            img_qnode = NULL; infoColor = NULL;
        }
    }
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



}  // namespace fencing_vision
