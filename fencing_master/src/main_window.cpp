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
#define NONE          0X00
#define FRONT         0XO1
#define BACK          0X02
#define LEFT          0X03
#define RIGHT         0X04
#define L_TURN        0X05
#define R_TURN        0X06
#define UP_ATTACK     0X07
#define DOWN_ATTACK   0X08
#define BOTH_L        0X09
#define BOTH_R        0X10
#define DEFENSE       0X11
#define UP            1
#define DOWN           2


#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/fencing_master/main_window.hpp"
#include <QKeyEvent>

/*****************************************************************************
** Namespaces
*****************************************************************************/

extern ros::Publisher fencing_master_to_Pub;
msg_generate::fencing_master_to fencing_master_to_Msg;
extern ros::Publisher motionNum_Pub;
msg_generate::motionNum_msg motion;

extern ros::Subscriber fencing_ui_to_Sub;
extern ros::Subscriber fencing_vision_to_Sub;
extern ros::Subscriber motion_end_Sub;
msg_generate::motion_end motion_end_Msg;
extern ros::Subscriber imu_Sub;

namespace fencing_master {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

bool playMotion(int motion_num);

//subscribe data
int _moving_mode;
int _action_mode;
int _current_moving;
int _current_action;

int _redx;
int _redy;
int _greenx;
int _greeny;
int _bluex;
int _bluey;

//int data
int location_red;
int location_green;
int location_blue;
int up = 0;
int down = 0;
int _target = 0;
int left = 0
int right = 0;
int _LR = 0;

//bool data
bool end_moving = false;
bool end_action = false;



MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    setWindowIcon(QIcon(":/images/icon.png"));

    qnode.init();

    m_Timer = new QTimer(this);
    connect(m_Timer, SIGNAL(timeout()), this, SLOT(main()));
    connect(m_Timer, SIGNAL(timeout()), this, SLOT(playMotion()));
    m_Timer->start(100);



    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Functions
*****************************************************************************/

void MainWindow::main(){

    bool on_red = false;
    bool on_green = false;
    bool on_blue = false;

    up = 0;
    down = 0;
    /********************************************************
                         VISION CALCULATE
    ********************************************************/

    //target detect
    if(_redx > 0 && _redx < 640){
        on_red = true;
        location_red = 0;
    }

    if(_greenx > 0 && _greenx < 640){
        on_green = true;
        location_green = 0;
    }

    if(_bluex > 0 && _bluex < 640){
        on_blue = true;
        location_blue = 0;
    }

    if(on_red == true){
        if(_redy > 0 && _redy < 240){
            location_red = UP;
            up++;
        }
        else if(_redy > 240 && _redy < 480){
            location_red = DOWN;
            down++;
        }
    }else{
        location_red = 0;
    }

    if(on_green == true){
        if(_greeny > 0 && _greeny < 240){
            location_green = UP;
            up++;
        }
        else if(_greeny > 240 && _greeny < 480){
            location_green = DOWN;
            down++;
        }
    }else{
        location_green = 0;
    }

    if(on_blue == true){
        if(_bluey > 0 && _bluey < 240){
            location_blue = UP;
            up++;
        }
        else if(_bluey > 240 && _bluey < 480){
            location_blue = DOWN;
            down++;
        }
    }else{
        location_blue = 0;
    }

    if(location_red = DOWN){
        if(_redx > 0 && _redx < 320)
            left++
        else
            right++;
    }

        if(location_green = DOWN){
        if(_greenx > 0 && _greenx < 320)
            left++
        else
            right++;
    }

        if(location_blue = DOWN){
        if(_bluex > 0 && _bluex < 320)
            left++
        else
            right++;
    }

    /********************************************************
                        MOTION CALCULATE
    ********************************************************/


    if(_moving_mode == FRONT){
        end_moving = playMotion(FRONT);
        _current_moving = 1;
        std::cout << "front launch" << std::endl;
    }else if(_moving_mode == BACK){
        end_moving = playMotion(BACK);
        _current_moving = 2;
        std::cout << "back launch" << std::endl;
    }else if(_moving_mode == LEFT){
        end_moving = playMotion(LEFT);
        _current_moving = 3;
        std::cout << "left launch" << std::endl;
    }else if(_moving_mode == RIGHT){
        end_moving = playMotion(RIGHT);
        _current_moving = 4;
        std::cout << "right launch" << std::endl;
    }else if(_moving_mode == L_TURN){
        end_moving = playMotion(L_TURN);
        _current_moving = 5;
        std::cout << "L_turn launch" << std::endl;
    }else if(_moving_mode == R_TURN){
        end_moving = playMotion(R_TURN);
        _current_moving = 6;
        std::cout << "R_turn launch" << std::endl;
    }

    if(up > down){
        _target = UP;
    }else if(up < down){
        _target = DOWN;
    }else if(up == down{
        _target = BOTH;    
    }

    if(up == 0 && down == 0){
        _target = 0;
    }

    if(_action_mode == 1){
        if(up > down){
            end_action = playMotion(UP_ATTACK);
            std::cout << "upper attack launch" << std::endl;
        }else if(up < down){
            end_action = playMotion(DOWN_ATTACK);
            std::cout << "down attack launch" << std::endl;
        }else{
            if(right > left)
                end_action = playMotion(BOTH_R);
            else
                end_action = playMotion(BOTH_L);
            
            std::cout << "both attack launch" << std::endl;
        }
    }else if(_action_mode == 2){
        end_action = playMotion(DEFENSE);
        std::cout << "defense launch" << std::endl;
    }
    /********************************************************
                        PUBLISH MESSEAGE
    ********************************************************/

    if(end_moving == true){
        end_moving = playMotion(0);
    }

        fencing_master_to_Msg.location_red = location_red;
        fencing_master_to_Msg.location_green = location_green;
        fencing_master_to_Msg.location_blue = location_blue;
        fencing_master_to_Msg.moving_mode = _moving_mode;
        fencing_master_to_Msg.action_mode = _action_mode;
        fencing_master_to_Msg.target = _target;
        fencing_master_to_Pub.publish(fencing_master_to_Msg);




}

void QNode::ui_callback(const msg_generate::fencing_ui_to::ConstPtr &msg){
    _moving_mode = msg->moving_mode;
    _action_mode = msg->action_mode;

}

void QNode::vision_callback(const msg_generate::fencing_vision_to::ConstPtr &msg){
    _redx = msg->redx;
    _redy = msg->redy;
    _greenx = msg->greenx;
    _greeny = msg->greeny;
    _bluex = msg->bluex;
    _bluey = msg->bluey;

}

bool playMotion(int motion_num)
{
    std::cout << "playMotion - ";
    motion.Motion_Mode = 1;
    motion.Motion_Num = motion_num;
    motionNum_Pub.publish(motion);

    bool motion_end = false;

    if(motion_end_Msg.motion_end) {
        motion_end = true;
    }
    return motion_end;
}

void QNode::imu_callback(const msg_generate::imu_msg::ConstPtr &msg){
    yaw = msg->yaw;
    pitch = msg->pitch;
    roll = msg->roll;

}

void QNode::motion_end_callback(const msg_generate::motion_end::ConstPtr &msg){
    motion_end = msg->motion_end;

}


}  // namespace fencing_master

