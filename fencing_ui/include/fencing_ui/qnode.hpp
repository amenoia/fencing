/**
 * @file /include/fencing_ui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef fencing_ui_QNODE_HPP_
#define fencing_ui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <msg_generate/fencing_vision_to.h>
#include <msg_generate/fencing_ui_to.h>
#include <msg_generate/fencing_master_to.h>
#include <QKeyEvent>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace fencing_ui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
        void run();

        cv::Mat* img_qnode;
        cv::Mat* infoColor;

        bool isRecv = false;

        image_transport::CameraSubscriber image_encodings_Sub;
        image_transport::CameraSubscriber image_sub;
        void imageCallBack(const sensor_msgs::ImageConstPtr& msg_img, const sensor_msgs::CameraInfoConstPtr& info_msg);

Q_SIGNALS:
    void rosShutdown();
    void recvImg(const cv::Mat& img, const cv::Mat& info);

private:
	int init_argc;
	char** init_argv;

    void callback(const msg_generate::fencing_vision_to::ConstPtr&);
    void master_callback(const msg_generate::fencing_master_to::ConstPtr&);

};

}  // namespace fencing_ui

#endif /* fencing_ui_QNODE_HPP_ */
