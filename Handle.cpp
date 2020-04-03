//
// Created by elsa on 2020/3/20.
//

#include "Handle.h"

Handle::Handle(Mat &src){
    image = src;
}

void Handle::setKeyPoints(vector<Point2f> pt2D, vector<vector<Point3f>> pt3D) {
    keyPoints2D = pt2D;
    keyPoints3D = pt3D;
}

void Handle::getPlane() {
    Point3f p1 = keyPoints3D[0][20]; //角点还是使用原点
    Point3f p2 = keyPoints3D[1][20];
    Point3f p3 = keyPoints3D[2][20];

    a = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
    b = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
    c = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
    d = 0 - (a * p1.x + b * p1.y + c * p1.z);

}

float Handle::fallPointOverflow(vector<Point3f> fallPoint) {
    getPlane();
    float distanceSum = 0;
    float distanceAvg = 0;
    for (int i = 0; i < fallPoint.size(); ++i) { //计算出落点一定范围内离平面的距离
        distanceSum +=  (a * fallPoint[i].x + b * fallPoint[i].y + c * fallPoint[i].z + d) / sqrt(a * a + b * b + c * c);
    }
    distanceAvg = distanceSum / fallPoint.size(); //取平均值
    return distanceAvg; //TODO 设置一个新的结构体，让与平面的距离成为落点的属性
}


void Handle::drawFallPoints(){ //TODO 没有画在原图上
    int maxHeight = 500;
    for (int i = 4; i < keyPoints3D.size(); ++i) {
        float height = fallPointOverflow(keyPoints3D[i]);
        if( height > maxHeight)
            circle(image, keyPoints2D[i], 2, cv::Scalar(0, 0, 255), 2);
        else if(height < maxHeight / 2)
            circle(image, keyPoints2D[i], 2, cv::Scalar(0, 255, 0), 2);
        else
            circle(image, keyPoints2D[i], 2, cv::Scalar(0, 255, 255), 2);
    }
}

