#include <iostream>
#include "Detection.h"
#include "Cam.h"
#include "Handle.h"

int main() {
    Cam zedCamera;
    zedCamera.cameraStart();
    cv::Mat image = zedCamera.getImage(0); //获取彩图

    Detection detection(image); //检测
    detection.process();
    detection.show();//怎么直接将原图传进去，修改也直接在原图上，就不需要show了

    vector<Point2f> keyPoints2D; //获取角点落点的像素坐标
    keyPoints2D = detection.getKeyPoints();
    vector<vector<Point3f>> keyPoints3D; //获取角点落点的点云图上的坐标
    keyPoints3D = zedCamera.get3DPoint(keyPoints2D);

    Handle handle(image); //怎么直接将原图传进去，修改也直接在原图上
    handle.setKeyPoints(keyPoints2D, keyPoints3D);


    return 0;
}


