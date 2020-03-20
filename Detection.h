//
// Created by elsa on 2020/3/18.
//

#ifndef FORAGEHARVEST_DETECTION_H
#define FORAGEHARVEST_DETECTION_H

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

    Vertex(int posX, int posY) : x(posX), y(posY) {}

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
    Mat srcImage;
    Mat dstImage;
    Point2f vertex2D[4];

    void HSVFilter(Mat inputImage, Mat &outputImage);

    void borderHough(Mat inputImage, Mat &outputImage);

    void getCrossPointAndIncrement(Vec4f LineA, Vec4f LineB, vector<Vertex> &vertexSet, int imgW, int imgH);

    void drawLines(vector<Vertex> top4vertexSet, Mat &outputImage);

    void drawBox(vector<Vertex> vertexSet, Mat &outputImage);

    void drawPoints(vector<Vertex> vertexSet, Mat &outputImage);

    void mostIntersections(vector<Vec4f> lines, vector<Vertex> &topVertexSet, int topVertexNum, int imgW, int imgH);

    void pointColor(Mat image, vector<Vertex> inputVertexSet, vector<Vertex> &outputVertexSet);

public:
    Detection(Mat image);

    ~Detection() {}

    void process();

    Point2f* getVertex();

    void show();

};


#endif //FORAGEHARVEST_DETECTION_H
