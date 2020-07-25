#include "Cam.h"
#include "Detection.h"
#include "Handle.h"
#include <iostream>

int main(int argc, char *argv[]) {

#if 1 // 工作
    Cam zedCamera;
    zedCamera.cameraStart();
    namedWindow("原始图像", WINDOW_NORMAL);
    while (1) {
        if (waitKey(10) == 27)
            break;
        cv::Mat image = zedCamera.getImage(0); //获取彩图
        imshow("原始图像", image);

        Detection detection(image); //检测
        detection.process();
        if (!detection.isExistLine()) {
            cout << "no lines" << endl;
            continue;
        }
        imshow("目标图像", image);

        // Point2i
        vector<Point2f> keyPoints2D; //获取角点+落点的像素坐标
        keyPoints2D = detection.getKeyPoints();
        vector<vector<Point3f>> keyPoints3D; //获取角点+落点的点云图上的坐标,落点周边范围
        keyPoints3D = zedCamera.get3DPoint(keyPoints2D);

        //        vector<Point2f> edgePoints2D; //获取角点+落点+边缘点的像素坐标
        //        edgePoints2D = detection.getEdgePoints();
        //        vector<vector<Point3f>> edgePoints3D; //获取角点+落点+边缘点的点云图上的坐标
        //        edgePoints3D = zedCamera.get3DPoint(edgePoints2D);

        Handle handle(image);
        handle.setKeyPoints(keyPoints2D, keyPoints3D);
        //        handle.setKeyPoints(edgePoints2D, edgePoints3D);
        handle.process();
    }

#elif 0 // color 测试
    Mat image = imread("../images/yellowBorder6.jpg");

    namedWindow("原始图像", WINDOW_NORMAL);
    resizeWindow("原始图像", 1000, 1000);
    imshow("原始图像", image);

    Detection detection(image); //检测
    detection.process();

    namedWindow("目标图像", WINDOW_NORMAL);
    resizeWindow("目标图像", 1000, 1000);
    imshow("目标图像", image);

    waitKey(0);

#elif 0 // depth 测试

    Mat colorImg = imread("../images/h2.png", IMREAD_COLOR);
    if (!colorImg.data) {
        cout << "读取图片错误" << endl;
    }
    Mat depthImg = imread("../images/depth-1.png");
    if (!depthImg.data) {
        cout << "读取图片错误" << endl;
    }

    Mat depthMap;
    depthImg.convertTo(depthMap, CV_32FC1, 18); // CV_8UC4 -> CV_32FC1  // alpha = 2^24?
    //    imshow("depth_map_ocv", depth_map_ocv);
    //    cout << depth_map_ocv << endl;

    namedWindow("深度图", WINDOW_NORMAL);
    resizeWindow("深度图", 1000, 1000);
    imshow("深度图", depthImg);

    Mat detectRes;
    Detection detection(colorImg, depthImg, depthMap); //检测
    detection.process_depth(detectRes);

    namedWindow("目标图像", WINDOW_NORMAL);
    resizeWindow("目标图像", 1000, 1000);
    imshow("目标图像", detectRes);

    waitKey(0);

#endif
//    zedCamera.cameraClose();
    return 0;
}
