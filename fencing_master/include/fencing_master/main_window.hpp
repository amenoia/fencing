/**
 * @file /include/fencing_master/main_window.hpp
 *
 * @brief Qt based gui for fencing_master.
 *
 * @date November 2010
 **/
#ifndef fencing_master_MAIN_WINDOW_H
#define fencing_master_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"


/*****************************************************************************
** Namespace
*****************************************************************************/

namespace fencing_master {

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

    QTimer* m_Timer;



public Q_SLOTS:

    void main();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace fencing_master

#endif // fencing_master_MAIN_WINDOW_H
