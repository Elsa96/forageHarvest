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

struct point3D {
    vector<Point3f> pt;
    int distance;

    point3D(vector<Point3f> _pt) {
        pt = _pt;
        distance = 0;
    }

    void setDistance(int _distance) {
        distance = _distance;
    }
};

class Handle {
private:
    Mat image;
    int armL, armR;
    vector<Point2f> keyPoints2D;
    vector<vector<Point3f>> keyPoints3D; //前4个是角点，后六个是落点
    vector<Point2f> fallPoints2D;
    vector<point3D> fallPoints3D; //带距离的落点
    vector<Point2f> edgePoints2D;
    vector<point3D> edgePoints3D; //带距离的落点
    float a, b, c, d; //平面的四个参数

    void getPlane(); //求取平面

    void fallPointOverflow(); //落点的满溢度

    void edgePointOverflow(); //边缘点的满溢度

    void drawFallPoints();

    void drawFallPoints_2();

    void right2Left();

public:
    Handle(Mat &src);

    ~Handle() {
    }

    void setKeyPoints(vector<Point2f> pt2D, vector<vector<Point3f>> pt3D);

    void process();
};

#endif // FORAGEHARVEST_HANDLE_H
