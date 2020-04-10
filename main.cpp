#include <iostream>
#include "Detection.h"
#include "Cam.h"
#include "Handle.h"

int main() {
    Cam zedCamera;
    zedCamera.cameraStart();
    cv::Mat image = zedCamera.getImage(0); //获取彩图
//    Mat image = imread("../images/yellowBorder14.jpg");

    namedWindow("原始图像", WINDOW_NORMAL);
    resizeWindow("原始图像", 1000, 1000);
    imshow("原始图像", image);


    Detection detection(image); //检测
    detection.process();

    namedWindow("目标图像", WINDOW_NORMAL);
    resizeWindow("目标图像", 1000, 1000);
    imshow("目标图像", image);



    vector<Point2f> keyPoints2D; //获取角点落点的像素坐标
    keyPoints2D = detection.getKeyPoints();
    vector<vector<Point3f>> keyPoints3D; //获取角点落点的点云图上的坐标
    keyPoints3D = zedCamera.get3DPoint(keyPoints2D);

    Handle handle(image); //怎么直接将原图传进去，修改也直接在原图上
    handle.setKeyPoints(keyPoints2D, keyPoints3D);

    waitKey(0);

    return 0;
}


