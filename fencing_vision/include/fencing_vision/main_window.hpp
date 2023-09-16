/**
 * @file /include/fencing_vision/main_window.hpp
 *
 * @brief Qt based gui for fencing_vision.
 *
 * @date November 2010
 **/
#ifndef fencing_vision_MAIN_WINDOW_H
#define fencing_vision_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <qtimer.h>
#include <opencv2/opencv.hpp>
#include "robit_master_vision.hpp"
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace fencing_vision {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

    QTimer *m_Timer;




    int hmin, hmax, smin, smax, vmin, vmax;
    int hmin2, hmax2, smin2, smax2, vmin2, vmax2;
    int hmin3, hmax3, smin3, smax3, vmin3, vmax3;

public Q_SLOTS:
    void update(const cv::Mat& img, const cv::Mat& info);

    void on_mode_clicked();
    void on_apply_clicked();
    void on_load_clicked();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace fencing_vision

#endif // fencing_vision_MAIN_WINDOW_H
