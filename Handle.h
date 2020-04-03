//
// Created by elsa on 2020/3/20.
//

#ifndef FORAGEHARVEST_HANDLE_H
#define FORAGEHARVEST_HANDLE_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;


class Handle {
private:
    Mat image;
    vector<Point2f> keyPoints2D;
    vector<vector<Point3f>> keyPoints3D; //前4个是角点，后六个是落点
    float a, b, c, d; //平面的四个参数

    void getPlane(); //求取平面

    float fallPointOverflow(vector<Point3f> fallPoint); //单个落点的满溢度

    void drawFallPoints();

    void move();


public:
    Handle(Mat &src);

    ~Handle() {}

    void setKeyPoints(vector<Point2f> pt2D, vector<vector<Point3f>> pt3D);

};


#endif //FORAGEHARVEST_HANDLE_H
