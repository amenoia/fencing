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

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/fencing_vision/main_window.hpp"
#include <opencv2/opencv.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/


using namespace std;

extern ros::Publisher fencing_vision_to_Pub;
msg_generate::fencing_vision_to fencing_vision_to_Msg;

namespace fencing_vision {

using namespace Qt;
using namespace cv;

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

int mode = 0;

int _upper_red[3] = {10, 255, 255};
int _lower_red[3] = {0 ,100, 100};

int _upper_green[3] = {105, 255, 166};
int _lower_green[3] = {78, 50, 0};

int _upper_blue[3] = {120, 255, 255};
int _lower_blue[3] = {110, 80, 47};

void MainWindow::update(const cv::Mat& img, const cv::Mat& info){
    int _redx = 0;
    int _redy = 0;
    int _greenx = 0;
    int _greeny = 0;
    int _bluex = 0;
    int _bluey = 0;



    hmin = ui.hmin->value();
    hmax = ui.hmax->value();
    smin = ui.smin->value();
    smax = ui.smax->value();
    vmin = ui.vmin->value();
    vmax = ui.vmax->value();

    hmin2 = ui.hmin_2->value();
    hmax2 = ui.hmax_2->value();
    smin2 = ui.smin_2->value();
    smax2 = ui.smax_2->value();
    vmin2 = ui.vmin_2->value();
    vmax2 = ui.vmax_2->value();

    hmin3 = ui.hmin_3->value();
    hmax3 = ui.hmax_3->value();
    smin3 = ui.smin_3->value();
    smax3 = ui.smax_3->value();
    vmin3 = ui.vmin_3->value();
    vmax3 = ui.vmax_3->value();

    Scalar lower_red = Scalar(hmin, smin, vmin);
    Scalar upper_red = Scalar(hmax, smax, vmax);

    Scalar lower_green = Scalar(hmin2, smin2, vmin2);
    Scalar upper_green = Scalar(hmax2, smax2, vmax2);

    Scalar lower_blue = Scalar(hmin3, smin3, vmin3);
    Scalar upper_blue = Scalar(hmax3, smax3, vmax3);

    if(mode == 1){
//        Scalar lower_red = Scalar(hmin, smin, vmin);
//        Scalar upper_red = Scalar(hmax, smax, vmax);

//        Scalar lower_green = Scalar(hmin2, smin2, vmin2);
//        Scalar upper_green = Scalar(hmax2, smax2, vmax2);

//        Scalar lower_blue = Scalar(hmin3, smin3, vmin3);
//        Scalar upper_blue = Scalar(hmax3, smax3, vmax3);

    cout << hmin << " " << hmax << " " << smin << " " << smax << " " << vmin << " " << vmax << endl;
    cout << hmin2 << " " << hmax2 << " " << smin2 << " " << smax2 << " " << vmin2 << " " << vmax2 << endl;
    cout << hmin3 << " " << hmax3 << " " << smin3 << " " << smax3 << " " << vmin3 << " " << vmax3 << endl;
    }else if(mode == 0){
        lower_red = Scalar(_lower_red[0], _lower_red[1], _lower_red[2]);
        upper_red = Scalar(_upper_red[0], _upper_red[1], _upper_red[2]);

        lower_green = Scalar(_lower_green[0], _lower_green[1], _lower_green[2]);
        upper_green = Scalar(_upper_green[0], _upper_green[1], _upper_green[2]);

       lower_blue = Scalar(_lower_blue[0], _lower_blue[1], _lower_blue[2]);
       upper_blue = Scalar(_upper_blue[0], _upper_blue[1], _upper_blue[2]);
}



    cv::Mat clone_mat = img.clone(); // make a clone img
    cv::GaussianBlur(clone_mat, clone_mat, Size(7, 7), 3); //gaussianblur
    cv::resize(clone_mat, clone_mat, cv::Size(640, 480), 0, 0, cv::INTER_CUBIC); //resize clone img

    cv::Mat binary_mat = clone_mat; //make a binary_img for map_labeling
    cv::cvtColor(clone_mat, binary_mat, COLOR_BGR2GRAY); //img convert BGR TO GRAY
    threshold(binary_mat,binary_mat,100,255,0); //thresholding img for making binary_img
    cv::resize(binary_mat, binary_mat, cv::Size(640, 480), 0, 0, cv::INTER_CUBIC); //resize binary_img

    /* MAP LABELING */

    cv::Mat clone_mat1 = img.clone();

    RobitLabeling labelField(binary_mat, 5000, 4); //maplabeling
    labelField.doLabeling();
    if(labelField.mergeArea(clone_mat)){ //if map area is detected
        labelField.eraseOutField(clone_mat1, clone_mat, 15, clone_mat1); //erase outfield of map
    }

    /* CONTOURING */

    //make a new Mat for contouring
    cv::Mat HSV_mat = clone_mat.clone();
    cv::Mat red_mat;
    cv::Mat green_mat;
    cv::Mat blue_mat;

    //img convert BGR to HSV for contouring
    cv::cvtColor(clone_mat, HSV_mat, COLOR_BGR2HSV);

    //img masking for contouring
    inRange(HSV_mat, lower_red, upper_red, red_mat);
    inRange(HSV_mat, lower_green, upper_green, green_mat);
    inRange(HSV_mat, lower_blue, upper_blue, blue_mat);

    //erode & dilate for removing noise
    cv::erode(red_mat, red_mat, cv::Mat(), cv::Point(-1, -1), 3);
    cv::dilate(red_mat, red_mat, cv::Mat(), cv::Point(-1, -1), 10);
    cv::erode(green_mat, green_mat, cv::Mat(), cv::Point(-1, -1), 3);
    cv::dilate(green_mat, green_mat, cv::Mat(), cv::Point(-1, -1), 10);
    cv::erode(blue_mat, blue_mat, cv::Mat(), cv::Point(-1, -1), 3);
    cv::dilate(blue_mat, blue_mat, cv::Mat(), cv::Point(-1, -1), 10);

        //finding contour for contouring
        vector<vector<Point>> contours_red;
        vector<vector<Point>> contours_green;
        vector<vector<Point>> contours_blue;

        vector<Vec4i> hierarchy_red;
        vector<Vec4i> hierarchy_green;
        vector<Vec4i> hierarchy_blue;

        cv::findContours(red_mat, contours_red, hierarchy_red, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        cv::findContours(green_mat, contours_green, hierarchy_green, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        cv::findContours(blue_mat, contours_blue, hierarchy_blue, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        //conPoly identified
        vector<vector<Point>> conPoly_red(contours_red.size());
        vector<vector<Point>> conPoly_green(contours_green.size());
        vector<vector<Point>> conPoly_blue(contours_blue.size());

        //draw countours
        drawContours(clone_mat, contours_red, -1, Scalar(0, 0, 255), 3);
        drawContours(clone_mat, contours_green, -1, Scalar(0, 255, 0), 3);
        drawContours(clone_mat, contours_blue, -1, Scalar(255, 0, 0), 3);





    //robitlabeling

    RobitLabeling green(green_mat, 5000, 4);
    green.doLabeling();
    green.sortingRecBlobs();

    if(green.m_nBlobs != 0){
        Point point1(green.m_recBlobs.at(0).x , green.m_recBlobs.at(0).y);
        Point point2(green.m_recBlobs.at(0).x + green.m_recBlobs.at(0).width , green.m_recBlobs.at(0).y + green.m_recBlobs.at(0).height);
        Size size(green.m_recBlobs.at(0).width , green.m_recBlobs.at(0).height);
        rectangle(green_mat, point1, point2, Scalar(0, 255, 0), 2, 8);
        rectangle(clone_mat, point1, point2, Scalar(0, 255, 0), 2, 8);
        circle(blue_mat, (point1 + point2) / 2, 5, Scalar(0, 0, 0));
        circle(clone_mat, (point1 + point2) / 2, 5, Scalar(0, 255, 0));
        _greenx = green.m_recBlobs.at(0).x + (green.m_recBlobs.at(0).width) / 2;
        _greeny = green.m_recBlobs.at(0).y + (green.m_recBlobs.at(0).height) / 2;
    }

    RobitLabeling red(red_mat, 5000, 4);
    red.doLabeling();
    red.sortingRecBlobs();

    if(red.m_nBlobs != 0){
        Point point1(red.m_recBlobs.at(0).x , red.m_recBlobs.at(0).y);
        Point point2(red.m_recBlobs.at(0).x + red.m_recBlobs.at(0).width , red.m_recBlobs.at(0).y + red.m_recBlobs.at(0).height);
        Size size(red.m_recBlobs.at(0).width , red.m_recBlobs.at(0).height);
        rectangle(red_mat, point1, point2, Scalar(0, 0, 255), 2, 8);
        rectangle(clone_mat, point1, point2, Scalar(0, 0, 255), 2, 8);
        circle(blue_mat, (point1 + point2) / 2, 5, Scalar(0, 0, 0));
        circle(clone_mat, (point1 + point2) / 2, 5, Scalar(0, 0, 255));
        _redx = red.m_recBlobs.at(0).x + (red.m_recBlobs.at(0).width) / 2;
        _redy = red.m_recBlobs.at(0).y + (red.m_recBlobs.at(0).height) / 2;
    }

    RobitLabeling blue(blue_mat, 5000, 4);
    blue.doLabeling();
    blue.sortingRecBlobs();

    if(blue.m_nBlobs != 0){
        Point point1(blue.m_recBlobs.at(0).x , blue.m_recBlobs.at(0).y);
        Point point2(blue.m_recBlobs.at(0).x + blue.m_recBlobs.at(0).width , blue.m_recBlobs.at(0).y + blue.m_recBlobs.at(0).height);
        Size size(blue.m_recBlobs.at(0).width , blue.m_recBlobs.at(0).height);
        rectangle(blue_mat, point1, point2, Scalar(255, 0, 0), 2, 8);
        rectangle(clone_mat, point1, point2, Scalar(255, 0, 0), 2, 8);
        circle(blue_mat, (point1 + point2) / 2, 5, Scalar(0, 0, 0));
        circle(clone_mat, (point1 + point2) / 2, 5, Scalar(255, 0, 0));
        _bluex = blue.m_recBlobs.at(0).x + (blue.m_recBlobs.at(0).width) / 2;
        _bluey = blue.m_recBlobs.at(0).y + (blue.m_recBlobs.at(0).height) / 2;
    }


    //countouring Mat summarize
    cv::Mat result_mat = red_mat + green_mat + blue_mat;

    //convert Mat to QImage for display a img
    QImage image = QImage((const unsigned char*)(clone_mat.data), clone_mat.cols, clone_mat.rows, QImage::Format_RGB888);
    ui.scene->setPixmap(QPixmap::fromImage(image.rgbSwapped()));
    QImage image2 = QImage((const unsigned char*)(result_mat.data), result_mat.cols, result_mat.rows, QImage::Format_Indexed8);
    ui.label->setPixmap(QPixmap::fromImage(image2.rgbSwapped()));

//    if(_redx < 0 && _redx > 640){
//        _redx = 0;
//    }

//    if(_greenx < 0 && _greenx > 640){
//        _redx = 0;
//    }

//    if(_bluex < 0 && _bluex > 640){
//        _redx = 0;
//    }

    fencing_vision_to_Msg.redx = _redx;
    fencing_vision_to_Msg.redy = _redy;
    fencing_vision_to_Msg.greenx = _greenx;
    fencing_vision_to_Msg.greeny = _greeny;
    fencing_vision_to_Msg.bluex = _bluex;
    fencing_vision_to_Msg.bluey = _bluey;
    fencing_vision_to_Pub.publish(fencing_vision_to_Msg);


    //delete image
    delete qnode.img_qnode;
    qnode.isRecv = false;
}

void MainWindow::on_mode_clicked(){
    if(mode ==0) mode = 1;
    else if(mode == 1) mode = 0;

    std::cout << "---------------------------------------------------------------------" << std::endl;
    std::cout << "mode : " << mode << std::endl;
    std::cout << "---------------------------------------------------------------------" << std::endl;
}

void MainWindow::on_apply_clicked(){
   _lower_red[0] = hmin;
   _upper_red[0] = hmax;
   _lower_red[1] = smin;
   _upper_red[1] = smax;
   _lower_red[2] = vmin;
   _upper_red[2] = vmax;

   _lower_green[0] = hmin2;
   _upper_green[0] = hmax2;
   _lower_green[1] = smin2;
   _upper_green[1] = smax2;
   _lower_green[2] = vmin2;
   _upper_green[2] = vmax2;

   _lower_blue[0] = hmin3;
   _upper_blue[0] = hmax3;
   _lower_blue[1] = smin3;
   _upper_blue[1] = smax3;
   _lower_blue[2] = vmin3;
   _upper_blue[2] = vmax3;
   std::cout <<"*********************************************************************************************" << std::endl;
   std::cout << "apply complete" << std::endl;
   std::cout <<"*********************************************************************************************" << std::endl;
}

void MainWindow::on_load_clicked(){
    ui.hmin->setValue(_lower_red[0]);
    ui.hmax->setValue(_upper_red[0]);
    ui.smin->setValue(_lower_red[1]);
    ui.smax->setValue(_upper_red[1]);
    ui.vmin->setValue(_lower_red[2]);
    ui.vmax->setValue(_upper_red[2]);

    ui.hmin_2->setValue(_lower_green[0]);
    ui.hmax_2->setValue(_upper_green[0]);
    ui.smin_2->setValue(_lower_green[1]);
    ui.smax_2->setValue(_upper_green[1]);
    ui.vmin_2->setValue(_lower_green[2]);
    ui.vmax_2->setValue(_upper_green[2]);

    ui.hmin_3->setValue(_lower_blue[0]);
    ui.hmax_3->setValue(_upper_blue[0]);
    ui.smin_3->setValue(_lower_blue[1]);
    ui.smax_3->setValue(_upper_blue[1]);
    ui.vmin_3->setValue(_lower_blue[2]);
    ui.vmax_3->setValue(_upper_blue[2]);
}

}  // namespace fencing_vision

