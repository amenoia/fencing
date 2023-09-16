/**
 * @file /include/fencing_master/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef fencing_master_QNODE_HPP_
#define fencing_master_QNODE_HPP_

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
#include <msg_generate/fencing_master_to.h>
#include <msg_generate/fencing_ui_to.h>
#include <msg_generate/fencing_vision_to.h>
#include <msg_generate/imu_msg.h>
#include <msg_generate/motionNum_msg.h>
#include <msg_generate/motion_end.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace fencing_master {

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

    bool playMotion(int motion_num);

    double yaw = 0;
    double pitch = 0;
    double roll = 0;

    int motion_end = 0;


Q_SIGNALS:
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;

    void ui_callback (const msg_generate::fencing_ui_to::ConstPtr&);
    void vision_callback (const msg_generate::fencing_vision_to::ConstPtr&);
    void imu_callback (const msg_generate::imu_msg::ConstPtr&);
    void motion_end_callback (const msg_generate::motion_end::ConstPtr&);

};

}  // namespace fencing_master

#endif /* fencing_master_QNODE_HPP_ */
