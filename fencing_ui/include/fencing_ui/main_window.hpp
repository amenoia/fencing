/**
 * @file /include/fencing_ui/main_window.hpp
 *
 * @brief Qt based gui for fencing_ui.
 *
 * @date November 2010
 **/
#ifndef fencing_ui_MAIN_WINDOW_H
#define fencing_ui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <opencv2/opencv.hpp>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace fencing_ui {

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



public Q_SLOTS:

    void update(const cv::Mat& img, const cv::Mat& info);
    void on_front_clicked();
    void on_back_clicked();
    void on_left_clicked();
    void on_right_clicked();
    void on_R_turn_clicked();
    void on_L_turn_clicked();
    void on_attack_clicked();
    void on_defense_clicked();

    

private:
	Ui::MainWindowDesign ui;
	QNode qnode;

protected:
	void keyPressEvent(QKeyEvent *event);
};

}  // namespace fencing_ui

#endif // fencing_ui_MAIN_WINDOW_H
