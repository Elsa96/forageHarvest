//
// Created by elsa on 2020/3/18.
//

#ifndef FORAGEHARVEST_DETECTION_H
#define FORAGEHARVEST_DETECTION_H

#include <algorithm>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>

using namespace std;
using namespace cv;

struct Vertex {
    int x;
    int y;
    int crossTimes = 0;

    Vertex(int posX, int posY) : x(posX), y(posY) {
    }

    void setXY(int _x, int _y) {
        x = _x;
        y = _y;
    }

    void addCrossTimes() {
        crossTimes++;
    }
};

class Detection {
private:
    Mat colorImage;
    Mat depthImage;
    Mat depthMapp;
    Mat srcImage;
    Mat dstImage;
    vector<Point2f> vertex2D; // 4个角点
    vector<Point2f> midFallPoint2D; // 6个落点 TODO 落点个数
    vector<int> midFallPointLevel; // 6个落点 颜色
    vector<Point2f> keyPoints; // 4个角点+6个落点

    vector<Point2f> edgePointsUp2D; // 6个上车斗边缘点
    vector<Point2f> edgePointsDown2D; // 6个下车斗边缘点
    vector<Point2f> edgePoints; // 4个角点+6个落点+12个边缘点

    bool isHasLine;

    void HSVFilter(Mat inputImage, Mat &outputImage); // TODO 参数有必要省略吗

    void borderHough(Mat inputImage, Mat &outputImage);

    void getCrossPointAndIncrement(Vec4f LineA, Vec4f LineB, vector<Vertex> &vertexSet, int imgW, int imgH);

    void drawLines(vector<Vertex> top4vertexSet, Mat &outputImage);

    void drawBox(vector<Vertex> vertexSet, Mat &outputImage);

    void drawPoints(vector<Vertex> vertexSet, Mat &outputImage);

    void mostIntersections(vector<Vec4f> lines, vector<Vertex> &topVertexSet, int topVertexNum, int imgW, int imgH);

    void pointColor(Mat image, vector<Vertex> inputVertexSet, vector<Vertex> &outputVertexSet);

    void midFallPointFind(); //绘制落点

    void edgePointFind(); //绘制车斗边缘的点，并反馈在落点上

    void drawArmRange(); //绘制饲料下落的范围，一直在屏幕的中心区域

    void greenMask(Mat colorImg, Mat &outMask);

    void getSrcImage(Mat &colorImage, Mat &depthImage, Mat &depthMap);

    void getROI(Mat inputGray, Mat &roiImage, Rect &roiBoundRect);

    double getPointMeanDepthVal(Point targetPoint);

    double getPointMaxDepthVal(Point targetPoint);

    void midFallPointOverflowLevel();

public:
    Detection(Mat &image);

    Detection(Mat &colorImg, Mat &depthImg, Mat &depthMap);

    ~Detection() {
    }

    void process();

    void process_depth(Mat &detectionRes);

    vector<Point2f> getKeyPoints(); // TODO 只检测到三个点的情况还需考虑

    vector<Point2f> getEdgePoints();

    bool isExistLine();
};

#endif // FORAGEHARVEST_DETECTION_H
