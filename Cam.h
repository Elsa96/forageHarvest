//
// Created by elsa on 2020/3/19.
//

#ifndef FORAGEHARVEST_CAM_H
#define FORAGEHARVEST_CAM_H

#include <sl/Camera.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace sl;
using namespace cv;
using namespace std;


class Cam {
private:
    Camera zed;
    InitParameters init_parameters;
    RuntimeParameters runtime_parameters;
    sl::Mat color, depth, pointCloud;
    sl::float4 pointCloudValue;
    vector<vector<Point3f>> keyPoints3D;

    cv::Mat slMat2cvMat(sl::Mat &input);

public:
    int cameraStart();

//    void intrinsic();

    cv::Mat getImage(int key);

    vector<vector<Point3f>> get3DPoint(vector<Point2f> keyPoints);

//    void getDistance(vector<Point2f> point, vector<float> &pointDistance); //点离相机的距离
};


#endif //FORAGEHARVEST_CAM_H
