/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/
//#define NONE 0
//#define UP 1
//#define DOWN 2
//#define FRONT 1
//#define BACK 2
//#define LEFT 3
//#define RIGHT 4
//#define L_TURN 5
//#define R_TURN 6
//#define UP_ATTACK 1
//#define DOWN_ATTACK 2
//#define DEFENSE 3

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/fencing_ui/main_window.hpp"
#include <opencv2/opencv.hpp>
#include <vector>

/*****************************************************************************
** Namespaces
*****************************************************************************/

extern ros::Subscriber fencing_master_to_Sub;

extern ros::Subscriber fencing_vision_to_Sub;

extern ros::Publisher fencing_ui_to_Pub;
msg_generate::fencing_ui_to fencing_ui_to_Msg;

using namespace std;
using namespace cv;

namespace fencing_ui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    setWindowIcon(QIcon(":/images/icon.png"));

    qnode.init();



    qRegisterMetaType<cv::Mat>("cv::Mat");
    connect(&qnode, SIGNAL(recvImg(const cv::Mat&, const cv::Mat&)), this, SLOT(update(const cv::Mat&, const cv::Mat&)));

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Functions
*****************************************************************************/

int _redx;
int _redy;
int _greenx;
int _greeny;
int _bluex;
int _bluey;

int _moving_mode = 0;
int _action_mode = 0;

int _current_moving = 0;
int _current_action = 0;
int _location_red = 0;
int _location_green = 0;
int _location_blue = 0;
int _target = 0;

int _moving_end = 0;
int _action_end = 0;

void MainWindow::update(const cv::Mat& img, const cv::Mat& info){
    ui.redx->setNum(_redx);
    ui.redy->setNum(_redy);
    ui.greenx->setNum(_greenx);
    ui.greeny->setNum(_greeny);
    ui.bluex->setNum(_bluex);
    ui.bluey->setNum(_bluey);

    if(_location_red == 0)
    ui.red_loc->setText("NO TARGET");
    else if(_location_red == 1)
    ui.red_loc->setText("UP");
    else if(_location_red == 2)
    ui.red_loc->setText("DOWN");

    if(_location_green == 0)
    ui.green_loc->setText("NO TARGET");
    else if(_location_green == 1)
    ui.green_loc->setText("UP");
    else if(_location_green == 2)
    ui.green_loc->setText("DOWN");

    if(_location_blue == 0)
    ui.blue_loc->setText("NO TARGET");
    else if(_location_blue == 1)
    ui.blue_loc->setText("UP");
    else if(_location_blue == 2)
    ui.blue_loc->setText("DOWN");

    if(_target == 1){
        ui.target->setText("UP");
    }else if(_target == 2){
        ui.target->setText("DOWN");
    }else{
        ui.target->setText("NO TARGET");
    }

    cv::Mat clone_mat = img.clone();
    cv::resize(clone_mat, clone_mat, cv::Size(640, 480), 0, 0, cv::INTER_CUBIC);
    cv::circle(clone_mat, Point(_redx, _redy), 5, Scalar(0, 0, 255));
    cv::circle(clone_mat, Point(_greenx, _greeny), 5, Scalar(0, 255, 0));
    cv::circle(clone_mat, Point(_bluex, _bluey), 5, Scalar(255, 0, 0));
    QImage image = QImage((const unsigned char*)(clone_mat.data), clone_mat.cols, clone_mat.rows, QImage::Format_RGB888);
    ui.scene->setPixmap(QPixmap::fromImage(image.rgbSwapped()));

    fencing_ui_to_Msg.moving_mode = _moving_mode;
    fencing_ui_to_Msg.action_mode = _action_mode;
    fencing_ui_to_Pub.publish(fencing_ui_to_Msg);


    _moving_mode = 0;
    _action_mode = 0;

    //delete image
    delete qnode.img_qnode;
    qnode.isRecv = false;
}

void QNode::callback(const msg_generate::fencing_vision_to::ConstPtr &msg){
    _redx = msg->redx;
    _redy = msg->redy;
    _greenx = msg->greenx;
    _greeny = msg->greeny;
    _bluex = msg->bluex;
    _bluey = msg->bluey;
}

void QNode::master_callback(const msg_generate::fencing_master_to::ConstPtr &msg){
    _current_action = msg->current_action;
    _current_moving = msg->current_moving;
    _location_red = msg->location_red;
    _location_green = msg->location_green;
    _location_blue = msg->location_blue;
    _target = msg->target;
    _moving_end = msg->moving_end;
    _action_end = msg->action_end;

}

void MainWindow::on_front_clicked(){
    _moving_mode = 1;
}

void MainWindow::on_back_clicked(){
    _moving_mode = 2;
}

void MainWindow::on_left_clicked(){
    _moving_mode = 3;
}

void MainWindow::on_right_clicked(){
    _moving_mode = 4;
}

void MainWindow::on_L_turn_clicked(){
    _moving_mode = 5;
}

void MainWindow::on_R_turn_clicked(){
    _moving_mode = 6;
}

void MainWindow::on_attack_clicked(){
    _action_mode = 1;
}

void MainWindow::on_defense_clicked(){
    _action_mode = 2;
}

void MainWindow::KeyPressEvent(QKeyEvent *event){
    QString text;
    switch(event->key()){
    case Qt::Key_W:
        _moving_mode = 1;
        std::cout << " 1 " << std::endl;
        break;
    case Qt::Key_S:
        _moving_mode = 2;
        std::cout << " 2 " << std::endl;
        break;
    case Qt::Key_A:
        _moving_mode = 3;
        std::cout << " 3 " << std::endl;
        break;
    case Qt::Key_D:
        _moving_mode = 4;
        std::cout << " 4 " << std::endl;
        break;
    case Qt::Key_Q:
        _moving_mode = 5;
        std::cout << " 5 " << std::endl;
        break;
    case Qt::Key_E:
        _moving_mode = 6;
        std::cout << " 6 " << std::endl;
        break;
    default:
        std::cout << " X " << std::endl;
    }
    fencing_ui_to_Msg.moving_mode = _moving_mode;
    fencing_ui_to_Msg.action_mode = _action_mode;
    fencing_ui_to_Pub.publish(fencing_ui_to_Msg);
}

}  // namespace fencing_ui

