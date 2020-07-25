//
// Created by elsa on 2020/3/18.
//

#include "Detection.h"

//色相,饱和度,亮度（黄色）
Scalar hsvMin = Scalar(26, 43, 46);
Scalar hsvMax = Scalar(34, 255, 255);

// cv::Mat hsvMin_mat = cv::Mat(hsvMin); //将vector变成单列的mat
// cv::Mat hsvMax_mat = cv::Mat(hsvMax); //将vector变成单列的mat

Detection::Detection(Mat &colorImg) {
    vertex2D.resize(4); // 分配空间,reserve是个坑
    midFallPoint2D.resize(6);
    midFallPointLevel.resize(6);
    edgePointsUp2D.resize(6);
    edgePointsDown2D.resize(6);
    srcImage = colorImg.clone();
    dstImage = colorImg;
    isHasLine = true;
}

Detection::Detection(Mat &colorImg, Mat &depthImg, Mat &depthMap) : Detection(colorImg) {
    this->colorImage = colorImg;
    this->depthImage = depthImg;
    this->depthMapp = depthMap;
}

void Detection::process() {
    Mat mask;
    HSVFilter(srcImage, mask);
    borderHough(mask, dstImage);
    if (!isHasLine)
        return;
    midFallPointFind();
    drawArmRange();
}

// 4个角点+6个中间落点 作为关键点
vector<Point2f> Detection::getKeyPoints() {
    // 将vec1和vec2的内容合并到keyPoints中
    keyPoints.clear();
    keyPoints.insert(keyPoints.end(), vertex2D.begin(), vertex2D.end());
    keyPoints.insert(keyPoints.end(), midFallPoint2D.begin(), midFallPoint2D.end());
    return keyPoints;
}

// 4个角点+6个中间落点+上下边缘点
vector<Point2f> Detection::getEdgePoints() {
    edgePoints.clear();
    edgePoints.insert(edgePoints.end(), vertex2D.begin(), vertex2D.end());
    edgePoints.insert(edgePoints.end(), midFallPoint2D.begin(), midFallPoint2D.end());
    edgePoints.insert(edgePoints.end(), edgePointsUp2D.begin(), edgePointsUp2D.end());
    edgePoints.insert(edgePoints.end(), edgePointsDown2D.begin(), edgePointsDown2D.end());
    return edgePoints;
}

// HSV预处理
void Detection::HSVFilter(Mat inputImage, Mat &outputImage) {
    Mat hsvImage;
    // bgr转hsv
    cvtColor(inputImage, hsvImage, CV_BGR2HSV);

    Mat mask;

    // scalar不是bgr吗，为什么可以限定上下限，Scalar只是一个不超过4维的向量
    inRange(hsvImage, hsvMin, hsvMax, mask);

    //形态学运算
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    erode(mask, mask, element); //腐蚀 TODO 直接改成闭运算？？
    dilate(mask, mask, element); //膨胀

    outputImage = mask;
    imshow("binary image", outputImage);
}

// 霍夫直线检测
void Detection::borderHough(Mat inputImage, Mat &outputImage) {
    vector<Vec4f> lines;
    HoughLinesP(inputImage, lines, 1, CV_PI / 180, 90, 50, 10); //第五个参数：超过这个值才被检测出直线
    //排除没有检测到直线的情况
    if (lines.empty()) {
        isHasLine = false;
        return;
    } else{
        cout << "lines number: " << lines.size() << endl;
    }

    int imgW = inputImage.cols;
    int imgH = inputImage.rows;

    // TODO 如何精简
    int gap = 50;
    //上下左右四条边界线端点
    Vec4f lineUp(gap, gap, imgW - gap, gap); // up
    Vec4f lineDown(gap, imgH - gap, imgW - gap, imgH - gap); // down
    Vec4f lineLeft(gap + 10, gap, gap, imgH - gap); // left 注意不要完全垂直
    Vec4f lineRight(imgW - gap - 10, gap, imgW - gap, imgH - gap); // right

    //将画面上下两条边加进去
    int linesNum = 20; // TODO 参数linesNum
    vector<Vec4f> lines3SidesUD(lines);
    lines3SidesUD.insert(lines3SidesUD.end(), linesNum, lineUp);
    lines3SidesUD.insert(lines3SidesUD.end(), linesNum, lineDown);
    //将画面左右两条边加进去
    vector<Vec4f> lines3SidesLR(lines);
    lines3SidesLR.insert(lines3SidesLR.end(), linesNum, lineLeft);
    lines3SidesLR.insert(lines3SidesLR.end(), linesNum, lineRight);
    //将画面上下左右都加进去
    vector<Vec4f> lines2Sides(lines3SidesUD);
    lines2Sides.insert(lines2Sides.end(), linesNum, lineLeft);
    lines2Sides.insert(lines2Sides.end(), linesNum, lineRight);

    Mat tempImage = outputImage.clone();

    vector<Vertex> top4vertexSet;
    mostIntersections(lines, top4vertexSet, 4, imgW, imgH);

    cout << "直线最大相交次数：" << top4vertexSet[0].crossTimes << endl;

    //判断有几条直线
    if (top4vertexSet[0].crossTimes > 4 * top4vertexSet[1].crossTimes) { // TODO 参数4
        vector<Vertex> top9vertexSet;
        cout << "只有两条直线，并相交" << endl; // TODO 只有三个交点的情况要忽略
        mostIntersections(lines2Sides, top9vertexSet, 9, imgW, imgH);
        vector<Vertex> vertexResult;
        pointColor(outputImage, top9vertexSet, vertexResult);

        for (int k = 0; k < vertexResult.size(); ++k) {
            vertex2D[k] = Point2f(vertexResult[k].x, vertexResult[k].y);
        }

        int findPoint4 = 0;
        int draw = 0;
        for (int i = 0; i < vertexResult.size(); ++i) {
            if (vertexResult[i].x < 2 * gap) {
                for (int j = 0; j < vertexResult.size(); ++j) {
                    if (j != i) {
                        if (vertexResult[j].x < 2 * gap) {
                            drawBox(vertexResult, outputImage);
                            findPoint4 = 1;
                            draw = 1;
                            break;
                        }
                        if (vertexResult[j].y < 2 * gap) {
                            Vertex newVertex(gap, gap);
                            vertexResult.push_back(newVertex);
                            findPoint4 = 1;
                            break;
                        }
                        if (vertexResult[j].y > imgH - 2 * gap) {
                            Vertex newVertex(gap, imgH - gap);
                            vertexResult.push_back(newVertex);
                            findPoint4 = 1;
                            break;
                        }
                    }
                }
            }
            if (vertexResult[i].x > imgW - 2 * gap) {
                for (int j = 0; j < vertexResult.size(); ++j) {
                    if (j != i) {
                        if (vertexResult[j].x > imgW - 2 * gap) {
                            drawBox(vertexResult, outputImage);
                            findPoint4 = 1;
                            draw = 1;
                            break;
                        }
                        if (vertexResult[j].y < 2 * gap) {
                            Vertex newVertex(imgW - gap, gap);
                            vertexResult.push_back(newVertex);
                            findPoint4 = 1;
                            break;
                        }
                        if (vertexResult[j].y > imgH - 2 * gap) {
                            Vertex newVertex(imgW - gap, imgH - gap);
                            vertexResult.push_back(newVertex);
                            findPoint4 = 1;
                            break;
                        }
                    }
                }
            }
            if ((vertexResult[i].y > imgH - 2 * gap) &&
                (vertexResult[(i + 1) % vertexResult.size()].y > imgH - 2 * gap)) {
                drawBox(vertexResult, outputImage);
                draw = 1;
                break;
            }

            if ((vertexResult[i].y < 2 * gap) && (vertexResult[(i + 1) % vertexResult.size()].y < 2 * gap)) {
                drawBox(vertexResult, outputImage);
                draw = 1;
                break;
            }
            if (findPoint4 == 1)
                break;
        }
        if (draw == 0) {
            drawLines(vertexResult, outputImage);
            drawPoints(vertexResult, outputImage);
        }
    } else if (top4vertexSet[0].crossTimes < 100) { // TODO 参数1000
        cout << "只有两条直线，平行" << endl;
        vector<Vertex> top4vertexSet_2;
        mostIntersections(lines2Sides, top4vertexSet_2, 4, imgW, imgH);
        if (top4vertexSet_2[0].crossTimes <= 4 * top4vertexSet_2[2].crossTimes) { //排除只有一条边的情况

            for (int k = 0; k < top4vertexSet_2.size(); ++k) {
                vertex2D[k] = Point2f(top4vertexSet_2[k].x, top4vertexSet_2[k].y);
            }

            drawLines(top4vertexSet_2, outputImage);
            drawPoints(top4vertexSet_2, outputImage);
        }

    } else if (top4vertexSet[0].crossTimes > 4 * top4vertexSet[2].crossTimes) { // TODO 参数4
        cout << "只有三条直线" << endl;
        vector<Vertex> top6vertexSet;
        int yGap = 500; // TODO 参数yGap
        if (abs(top4vertexSet[0].y - top4vertexSet[1].y) < yGap) //若两交点y很接近，则取上下边界
            mostIntersections(lines3SidesUD, top6vertexSet, 6, imgW, imgH);
        else
            mostIntersections(lines3SidesLR, top6vertexSet, 6, imgW, imgH);

        vector<Vertex> vertexResult;
        pointColor(outputImage, top6vertexSet, vertexResult);

        for (int k = 0; k < vertexResult.size(); ++k) {
            vertex2D[k] = Point2f(vertexResult[k].x, vertexResult[k].y);
        }

        cout << "交点个数为：" << vertexResult.size() << endl;
        //绘制直线
        drawLines(vertexResult, outputImage);
        //绘制交点
        drawPoints(vertexResult, outputImage);
    } else {
        cout << "四条直线" << endl;
        for (int k = 0; k < top4vertexSet.size(); ++k) {
            vertex2D[k] = Point2f(top4vertexSet[k].x, top4vertexSet[k].y);
        }
        //绘制直线
        drawLines(top4vertexSet, outputImage);
        //绘制最终的四个交点
        drawPoints(top4vertexSet, outputImage);
    }
}

// 求直线交点
void Detection::getCrossPointAndIncrement(Vec4f LineA, Vec4f LineB, vector<Vertex> &vertexSet, int imgW, int imgH) {
    float ka, kb;
    ka = (LineA[3] - LineA[1]) / (LineA[2] - LineA[0]); //求出LineA斜率
    kb = (LineB[3] - LineB[1]) / (LineB[2] - LineB[0]); //求出LineB斜率

    Point2f crossPoint;
    crossPoint.x = (ka * LineA[0] - LineA[1] - kb * LineB[0] + LineB[1]) / (ka - kb);
    crossPoint.y = (ka * kb * (LineA[0] - LineB[0]) + ka * LineB[1] - kb * LineA[1]) / (ka - kb);

    int x = (int) (round)(crossPoint.x);
    int y = (int) (round)(crossPoint.y);

    int VertexGap = 40000; // TODO VerTexGap

    if (x >= -imgW / 2 && x <= imgW * 1.5 && y >= -imgH / 2 && y <= imgH * 1.5) { //在图像区域内
        int i = 0;
        for (i = 0; i < vertexSet.size(); i++) { //与已有的点靠近，可以合并
            int oldX = vertexSet[i].x;
            int oldY = vertexSet[i].y;

            //附近有特别靠近的点，可以合并
            if ((oldX - x) * (oldX - x) + (oldY - y) * (oldY - y) <= VertexGap) {
                vertexSet[i].addCrossTimes();
                break;
            }
        }

        if (i == vertexSet.size()) { //如果该点附近没有距离特别近的点，自身作为一个新点
            Vertex newVertex(x, y);
            vertexSet.push_back(newVertex);
        }
    }
}

// 直线相交次数最多的几个点
void Detection::mostIntersections(vector<Vec4f> lines, vector<Vertex> &topVertexSet, int topVertexNum, int imgW,
                                  int imgH) {
    //获取所有直线的交点和相交次数
    vector<Vertex> vertexSet;
    for (unsigned int i = 0; i < lines.size(); i++) {
        for (unsigned int j = i + 1; j < lines.size(); j++) {
            getCrossPointAndIncrement(lines[i], lines[j], vertexSet, imgW, imgH);
        }
    }

    //找相交次数最多的topVertexNum个点
    // vertexSet 按照crossTimes排序，取前topVertexNum个
    sort(vertexSet.begin(), vertexSet.end(),
         [](const Vertex &vt1, const Vertex &vt2) { return vt1.crossTimes > vt2.crossTimes; }); // 降序
    topVertexSet.assign(vertexSet.begin(), vertexSet.begin() + topVertexNum); //取前topVertexNum个
}

// 验证角点是否为黄色 verifyVertexColor
void Detection::pointColor(Mat image, vector<Vertex> inputVertexSet, vector<Vertex> &outputVertexSet) {
    int imgW = image.cols;
    int imgH = image.rows;

    Mat hsvImage;
    cvtColor(image, hsvImage, CV_BGR2HSV); // bgr转hsv

    for (int i = 0; i < inputVertexSet.size(); ++i) {
        int x = inputVertexSet[i].x;
        int y = inputVertexSet[i].y;
        // TODO 如何精简
        if (x < 0)
            inputVertexSet[i].x = 0;
        if (x > imgW)
            inputVertexSet[i].x = imgW;
        if (y < 0)
            inputVertexSet[i].y = 0;
        if (y > imgH)
            inputVertexSet[i].y = imgH;
        int range = 200; // TODO 参数range
        int flag = 0;
        //注意不要超出画面边界
        int xMin = x - range;
        int xMax = x + range;
        int yMin = y - range;
        int yMax = y + range;
        if (xMin < 0)
            xMin = 0;
        if (xMin > imgW)
            xMin = imgW - 10;
        if (xMax < 0)
            xMax = 10;
        if (xMax > imgW)
            xMax = imgW;
        if (yMin < 0)
            yMin = 0;
        if (yMin > imgH)
            yMin = imgH - 10;
        if (yMax < 0)
            yMax = 10;
        if (yMax > imgH)
            yMax = imgH;
        //看交点是否为黄色
        for (int j = xMin; j < xMax; j = j + 5) {
            for (int k = yMin; k < yMax; k = k + 5) {
                cv::Vec3b hsvPoint = hsvImage.at<Vec3b>(k, j);
                cv::Vec3b hsvInRangeRes;
                inRange(hsvPoint, hsvMin, hsvMax, hsvInRangeRes); // 在范围内255，否则0
                int InRangeNum = countNonZero(hsvInRangeRes); // 非零个数
                if (InRangeNum == 3) { // hsv 都在范围内 返回（255,255,255）
                    outputVertexSet.push_back(inputVertexSet[i]);
                    flag = 1;
                    break;
                }
            }
            if (flag == 1)
                break;
        }
    }
}

// 框内中间落点坐标 midFallPointFind
void Detection::midFallPointFind() {
    sort(vertex2D.begin(), vertex2D.end(), [](Point2f &pt1, Point2f &pt2) { return pt1.y < pt2.y; }); // 按y值升序
    if (vertex2D[0].x > vertex2D[1].x) // 先比较y值最小的两个点的x值
        swap(vertex2D[0], vertex2D[1]);
    if (vertex2D[2].x > vertex2D[3].x)
        swap(vertex2D[2], vertex2D[3]);

    cout << "4个角点坐标" << endl;
    for (int i = 0; i < 4; i++)
        cout << "[ " << vertex2D[i].x << " , " << vertex2D[i].y << " ]" << endl;

    Point2f midPointL = Point2f((vertex2D[0].x + vertex2D[2].x) / 2, (vertex2D[0].y + vertex2D[2].y) / 2);
    Point2f midPointR = Point2f((vertex2D[1].x + vertex2D[3].x) / 2, (vertex2D[1].y + vertex2D[3].y) / 2);

    float midK = (midPointR.y - midPointL.y) / (midPointR.x - midPointL.x);
    float midB = midPointL.y - midK * midPointL.x;

    int space = (midPointR.x - midPointL.x) / 7; //间距
    int fallPointStart = midPointR.x - space; //第一个落点在最右
    int fallPointNum = 6;
    for (int i = 0; i < fallPointNum; ++i) {
        float x = fallPointStart - i * space; //落点从右往左数123456
        float y = midK * x + midB;
        // midFallPoint2D
        midFallPoint2D.push_back(Point2f(x, y));
        circle(dstImage, Point2f(x, y), 30, cv::Scalar(0, 255, 0), -1);
    }
}

void Detection::edgePointFind() {
    sort(vertex2D.begin(), vertex2D.end(),
         [](const Point2f &pt1, const Point2f &pt2) { return pt1.y < pt2.y; }); // 升序

    //左上0，右上1，左下2，右下3
    if (vertex2D[0].x > vertex2D[1].x) // 先比较y值最小的两个点的x值
        swap(vertex2D[0], vertex2D[1]);
    if (vertex2D[2].x > vertex2D[3].x)
        swap(vertex2D[2], vertex2D[3]);

    int gap = 20;
    int pointNum = 6;

    Point2f upPointL = Point2f(vertex2D[0].x + gap, vertex2D[0].y + gap);
    Point2f upPointR = Point2f(vertex2D[1].x - gap, vertex2D[1].y + gap);

    float upK = (upPointR.y - upPointL.y) / (upPointR.x - upPointL.x);
    float upB = upPointL.y - upK * upPointL.x;

    int upSpace = (upPointR.x - upPointL.x) / 7; //间距
    int upStart = upPointR.x - upSpace; //第一个点在最右

    for (int i = 0; i < pointNum; ++i) {
        float x = upStart - i * upSpace; //从右往左数123456
        float y = upK * x + upB;
        edgePointsUp2D.push_back(Point2f(x, y));
    }

    Point2f downPointL = Point2f(vertex2D[2].x + gap, vertex2D[2].y - gap);
    Point2f downPointR = Point2f(vertex2D[3].x - gap, vertex2D[3].y - gap);

    float downK = (downPointR.y - downPointL.y) / (downPointR.x - downPointL.x);
    float downB = downPointL.y - downK * downPointL.x;

    int downSpace = (downPointR.x - downPointL.x) / 7; //间距
    int downStart = downPointR.x - downSpace; //第一个点在最右

    for (int i = 0; i < pointNum; ++i) {
        float x = downStart - i * downSpace; //从右往左数123456
        float y = downK * x + downB;
        edgePointsDown2D.push_back(Point2f(x, y));
    }
}

// 画角点 drawVertexPoints
void Detection::drawPoints(vector<Vertex> vertexSet, Mat &outputImage) {
    for (int i = 0; i < vertexSet.size(); i++) {
        //        cout << "(" << vertexSet[i].x << "," << vertexSet[i].y << ")" << vertexSet[i].crossTimes << endl;
        circle(outputImage, Point(vertexSet[i].x, vertexSet[i].y), 30, Scalar(0, 0, 255), -1);
    }
}

// 画边框线
void Detection::drawLines(vector<Vertex> top4vertexSet, Mat &outputImage) {
    int crossPoint = 0;
    for (int i = 1; i < 4; i++) { //第0个点与第i个点连线
        double temp_k =
                (double) (top4vertexSet[i].y - top4vertexSet[0].y) / (double) (top4vertexSet[i].x - top4vertexSet[0].x);
        double temp_b = (double) top4vertexSet[0].y - temp_k * (double) top4vertexSet[0].x;

        int flag = 0; //标志为正还是为负
        for (int j = 1; j < 4; j++) {
            if (j != i) {
                //第j个点的y坐标减线上坐标
                double diff = (double) top4vertexSet[j].y - (temp_k * (double) top4vertexSet[j].x + temp_b);
                if (flag == 0) {
                    flag = diff > 0 ? 1 : -1;
                } else {
                    if (flag == 1 && diff <= 0 || flag == -1 && diff > 0) {
                        crossPoint = i;
                        break;
                    }
                }
            }
        }
        if (crossPoint != 0)
            break;
    }

    for (int i = 1; i < 4; i++) {
        if (i != crossPoint) {
            line(outputImage, Point(top4vertexSet[i].x, top4vertexSet[i].y),
                 Point(top4vertexSet[0].x, top4vertexSet[0].y), Scalar(0, 255, 0), 30, LINE_AA);
            line(outputImage, Point(top4vertexSet[i].x, top4vertexSet[i].y),
                 Point(top4vertexSet[crossPoint].x, top4vertexSet[crossPoint].y), Scalar(0, 255, 0), 30, LINE_AA);
        }
    }
}

// 画三角形
void Detection::drawBox(vector<Vertex> vertexSet, Mat &outputImage) {
    cout << "-----------------" << endl;
    for (int i = 0; i < vertexSet.size(); i++) {
        Point pt = Point(vertexSet[i].x, vertexSet[i].y);
        Point pt1 = Point(vertexSet[(i + 1) % 3].x, vertexSet[(i + 1) % 3].y);
        line(outputImage, pt, pt1, Scalar(0, 255, 0), 30);
    }
    for (int i = 0; i < vertexSet.size(); ++i) {
        Point pt = Point(vertexSet[i].x, vertexSet[i].y);
        circle(outputImage, pt, 30, Scalar(0, 0, 255), -1);
    }
}

void Detection::drawArmRange() {
    int armHeight = srcImage.rows;
    int armL = srcImage.cols / 2 - srcImage.cols / 16; //左边界 //TODO 参数16
    int armR = srcImage.cols / 2 + srcImage.cols / 16; //右边界

    line(dstImage, Point(armL, 0), Point(armL, armHeight / 4), Scalar(0, 255, 0), 20, CV_AA); //画左边的饲料下落边界
    line(dstImage, Point(armR, 0), Point(armR, armHeight / 4), Scalar(0, 255, 0), 20, CV_AA); //画右边的饲料下落边界
}

void Detection::process_depth(Mat &detectionRes) {

    Mat roiImg;
    Rect roiRect;
    getROI(depthImage, roiImg, roiRect);
    //    imshow("getROI", roiImg);

    detectionRes = roiImg;

    Mat greMask;
    greenMask(colorImage, greMask);
}

void Detection::getSrcImage(Mat &colorImg, Mat &depthImg, Mat &depthMap_) {
    this->colorImage = colorImg;
    this->depthImage = depthImg;
    this->depthMapp = depthMap_;
}

// todo 预设参数
float minDist = 900;
float maxDist = 4000;

// 深度图获取ROI
void Detection::getROI(Mat inputGray, Mat &roiImage, Rect &roiBoundRect) {
    // todo 根据map和depth可以算出缩放比例，得实际距离与灰度值比，而不需要直接操作map
    // 在map图根据实际工作距离截取ROI
    Mat depth_map = depthMapp;
    //    medianBlur(depth_map, depth_map, 9); // 只能CV_8U
    Mat mapDistMask; // CV_8U
    inRange(depth_map, minDist, maxDist, mapDistMask);
    imshow("mapDistMask", mapDistMask);

    // todo 落柱引起的边缘可以填充为缺失值统一处理

    Mat grayImage; // 输出

    // 转成灰度
    if (inputGray.type() != CV_8UC1) {
        cvtColor(inputGray, grayImage, CV_BGR2GRAY);
    } else {
        grayImage = inputGray;
    }

    // 只保留在范围内的的像素值，不在map范围内的被当作缺失值处理
    grayImage = grayImage & mapDistMask; // 按位与
    //    cv::bitwise_and(grayImage, mapDistMask, grayImage); // 按位与,可加掩膜
    //    imshow("grayImage", grayImage);

    // 去除缺失值
    // 得到所有缺失值坐标 //二值化,七种常见阈值分割
    Mat lostPointImg;
    threshold(grayImage, lostPointImg, 1, 255, CV_THRESH_BINARY_INV);
    //    imshow("lostPointImg", lostPointImg);

    // 膨胀扩大缺失的边缘
    int dilation_size = 2;
    Mat element = getStructuringElement(MORPH_RECT, Size(2 * dilation_size + 1, 2 * dilation_size + 1));
    dilate(lostPointImg, lostPointImg, element);
    //    imshow("Dilation", lostPointImg);

    // 缺失值的点坐标
    vector<Point> lostPoint;
    // 查找非零的值
    findNonZero(lostPointImg, lostPoint);
    //    cout << "--lostPoint.size() " << lostPoint.size() << endl;

    // 中值滤波
    //    Mat medianBlurImage;
    medianBlur(grayImage, grayImage, 9); //第三个参数为int类型的ksize，必须为大于1的奇数（3、5、7....）
    //    imshow("中值滤波", grayImage);

    Mat cannyImg;
    // 用Canny算子检测边缘
    Canny(grayImage, cannyImg, 10, 30, 3);
    imshow("Canny", cannyImg);

    Mat binaryImg = cannyImg;
    //    // 去除缺失值引起的边缘，这一步应该就可以了，不需要这么麻烦，但是测试图有点坑
    //    binaryImg = cannyImg - lostPointImg;
    //    imshow("binaryImg", binaryImg);

    // 仅调试显示
    Mat contoursImg = Mat::zeros(inputGray.size(), CV_8UC3);

    vector<vector<Point>> contours; // 找到的各个轮廓的点,二维
    vector<Vec4i> hierarchy; // 轮廓层次结构
    // 寻找轮廓
    // 参数：二值图,找到的轮廓点,轮廓层次结构，轮廓的检索模式，轮廓估计的方法，偏移
    findContours(binaryImg, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0));
    // 对每个轮廓计算其凸包
    vector<vector<Point>> hull(contours.size());
    for (int i = 0; i < contours.size(); i++) {
        convexHull(Mat(contours[i]), hull[i], false);
    }
    // 绘出各个轮廓
    for (int i = 0; i < contours.size(); i++) {
        Scalar color = Scalar(255, 255, 255); //任意值
        //每个轮廓
        drawContours(contoursImg, contours, i, color, 1, 8, hierarchy, 0, Point());
        //每个轮廓凸包
        //        drawContours(contoursImg, hull, i, color, 1, 8, hierarchy, 0, Point());
    }
    imshow("ROI origin 轮廓", contoursImg);

    // 绘制各个轮廓的最小矩形
    vector<Rect> contourBound(contours.size()); // 定义轮廓外接矩形集合
    vector<RotatedRect> contMinBound(contours.size()); // 定义轮廓最小外接矩形集合
    Point2f minRectVertex[4]; // 矩形的四个端点Point
    for (int i = 0; i < contours.size(); i++) { // 对每个轮廓
        contourBound[i] = boundingRect(Mat(contours[i])); //计算每个轮廓正外接矩形
        contMinBound[i] = minAreaRect(Mat(contours[i])); //计算每个轮廓最小外接矩形
        contMinBound[i].points(minRectVertex); //把最小外接矩形四个端点复制给rect数组
        //绘制每个轮廓正外接矩形
        rectangle(contoursImg, Point(contourBound[i].x, contourBound[i].y),
                  Point(contourBound[i].x + contourBound[i].width, contourBound[i].y + contourBound[i].height),
                  Scalar(55, 55, 55), 1, 8);
    }

    //绘制所有轮廓点的正外接矩形
    // v1,v2排序,先按x排序，x相同的按y排，升序
    sort(lostPoint.begin(), lostPoint.end(),
         [](Point &p1, Point &p2) { return p1.x == p2.x ? p1.y < p2.y : p1.x < p2.x; }); // 升序
    vector<Point> allContoursPoints; // 所有轮廓的点
    cout << "--contours.size() " << contours.size() << endl;
    //各个轮廓的点合并到一起
    for (vector<vector<Point>>::iterator it = contours.begin(); it != contours.end(); ++it) {
        cout << "contours it->size() " << it->size() << endl;
        sort(it->begin(), it->end(),
             [](Point &p1, Point &p2) { return p1.x == p2.x ? p1.y < p2.y : p1.x < p2.x; }); // 升序
        //        for_each(it->begin(), it->end(), [](Point &p) { cout << p; });
        vector<Point> intersectionPoint; // 轮廓点与缺失点的交集
        intersectionPoint.clear();
        std::set_intersection(lostPoint.begin(), lostPoint.end(), it->begin(), it->end(),
                              insert_iterator<vector<Point>>(intersectionPoint, intersectionPoint.begin()),
                              [](Point &p1, Point &p2) { return p1.x == p2.x ? p1.y < p2.y : p1.x < p2.x; });
        cout << "intersectionPoint.size() " << intersectionPoint.size() << endl;
        //        for_each(intersectionPoint.begin(), intersectionPoint.end(), [](Point &p) { cout << p; });
        if (4 < it->size() && intersectionPoint.size() < (it->size()) * 0.3) { // todo x,0.3
            allContoursPoints.insert(allContoursPoints.end(), it->begin(), it->end());
            Rect validContBound = boundingRect(Mat(*it)); //计算每个轮廓正外接矩形
            //绘制每个轮廓正外接矩形
            rectangle(contoursImg, Point(validContBound.x, validContBound.y),
                      Point(validContBound.x + validContBound.width, validContBound.y + validContBound.height),
                      Scalar(0, 255, 255), 1, 8);
        }
    }
    // 筛选后的轮廓边框
    Rect allContoursBound = boundingRect(allContoursPoints);

    // ROI矩形偏移
    int offset = 20; // todo 20 默认矩形偏移
    allContoursBound = allContoursBound + Point(-offset, -offset) + Size(offset * 2, offset * 2); // 平移，缩放
    Rect allImgBound = Rect(0, 0, inputGray.cols, inputGray.rows);
    roiBoundRect = allImgBound & allContoursBound; // 矩形交集
    //设置ROI图，！注意是共享内存的方式
    roiImage = inputGray(roiBoundRect); // 共享内存
    // 绘制ROI矩形
    rectangle(contoursImg, roiBoundRect, Scalar(255, 255, 0), 2, 8);
    imshow("ROI 轮廓矩形", contoursImg);
    imshow("ROI", roiImage);
}

double Detection::getPointMeanDepthVal(Point targetPoint) {
    int gap = 20;
    Rect fallPointRngRec = Rect(targetPoint.x - gap / 2, targetPoint.y - gap / 2, gap, gap);
    Mat fallPointRange = depthImage(fallPointRngRec).clone(); // 不共享内存
    medianBlur(fallPointRange, fallPointRange, 7);
    Scalar meanDepthScalar = cv::mean(fallPointRange);
    double realDepthVal = meanDepthScalar.val[0]; // 真实值
    return realDepthVal;
}

double Detection::getPointMaxDepthVal(Point targetPoint) {
    int gap = 20;
    Rect fallPointRngRec = Rect(targetPoint.x - gap / 2, targetPoint.y - gap / 2, gap, gap);
    Mat fallPointRange = depthImage(fallPointRngRec).clone(); // 不共享内存
    medianBlur(fallPointRange, fallPointRange, 7);
    //对Mat进行赋值和其他操作
    double maxVal, minVal; // 必须double？？
    Point min_loc, max_loc;
    cv::minMaxLoc(fallPointRange, &minVal, &maxVal, &min_loc, &max_loc);
    return maxVal;
}

void Detection::midFallPointOverflowLevel() {
    // todo 这个排序应该前移到找到角点时
    sort(vertex2D.begin(), vertex2D.end(), [](Point2f &pt1, Point2f &pt2) { return pt1.y < pt2.y; }); // 按y值升序
    if (vertex2D[0].x > vertex2D[1].x) // 先比较y值最小的两个点的x值
        swap(vertex2D[0], vertex2D[1]);
    if (vertex2D[2].x > vertex2D[3].x)
        swap(vertex2D[2], vertex2D[3]);

    Point upLineVertexL = vertex2D[0];
    Point upLineVertexR = vertex2D[1];
    Point dwLineVertexL = vertex2D[2];
    Point dwLineVertexR = vertex2D[3];

    // upLine
    float upLineK = (upLineVertexR.y - upLineVertexL.y) / (upLineVertexR.x - upLineVertexL.x);
    float upLineB = upLineVertexL.y - upLineK * upLineVertexL.x;
    // dwLine
    float dwLineK = (dwLineVertexR.y - dwLineVertexL.y) / (dwLineVertexR.x - dwLineVertexL.x);
    float dwLineB = dwLineVertexL.y - upLineK * dwLineVertexL.x;
    int fallPointNum = 6;
    int upStep = (upLineVertexR.x - upLineVertexL.x) / fallPointNum; //间距
    int dwStep = (dwLineVertexR.x - dwLineVertexL.x) / fallPointNum; //间距

    //第一个落点在最右
    for (int i = 0; i < fallPointNum; ++i) {
        // upLine
        int upPos_x1 = upLineVertexR.x - i * upStep; //落点从右往左数123456
        int upPos_y1 = upLineK * upPos_x1 + upLineB;
        Point upPos1 = Point(upPos_x1, upPos_y1);
        double upVal1 = getPointMaxDepthVal(upPos1);

        int upPos_x2 = upLineVertexR.x - (i + 1) * upStep; //落点从右往左数123456
        int upPos_y2 = upLineK * upPos_x2 + upLineB;
        Point upPos2 = Point(upPos_x2, upPos_y2);
        double upVal2 = getPointMaxDepthVal(upPos2);

        // dwLine
        int dwPos_x1 = dwLineVertexR.x - i * dwStep; //落点从右往左数123456
        int dwPos_y1 = dwLineK * dwPos_x1 + dwLineB;
        Point dwPos1 = Point(dwPos_x1, dwPos_y1);
        double dwVal1 = getPointMaxDepthVal(dwPos1);

        int dwPos_x2 = dwLineVertexR.x - (i + 1) * dwStep; //落点从右往左数123456
        int dwPos_y2 = dwLineK * dwPos_x2 + dwLineB;
        Point dwPos2 = Point(dwPos_x2, dwPos_y2);
        double dwVal2 = getPointMaxDepthVal(dwPos2);

        double theoryDepthVal = ((upVal1 + upVal2) / 2 + (dwVal1 + dwVal2) / 2) / 2; //中间落点 理论值

        Point midFallPointPos = Point(((upPos1.x + upPos2.x) / 2 + (dwPos1.x + dwPos2.x) / 2) / 2,
                                      ((upPos1.y + upPos2.y) / 2 + (dwPos1.y + dwPos2.y) / 2) / 2);
        midFallPoint2D.push_back(midFallPointPos); // 记录中间落点坐标

        double realDepthVal = getPointMeanDepthVal(midFallPointPos); //中间落点 实际值

        double diffDepthVal = realDepthVal - theoryDepthVal;
        int overflowLevel = 0;
        if (diffDepthVal < 0) {
            overflowLevel = 2; // 红色
        } else if (diffDepthVal > 3) { // todo 以车宽为参考/map的绝对值
            overflowLevel = 1; // 黄色
        } else {
            overflowLevel = 0; // 绿色
        }
        midFallPointLevel.push_back(overflowLevel); // 记录中间落点 满溢度结果

        // 显示落点
        circle(dstImage, midFallPointPos, 30, cv::Scalar(0, 255, 0), 3);
    }
}

void Detection::greenMask(Mat colorImg, Mat &outMask) {
    //  标准 HSV 范围： (100, 30, 10) -- (160, 70, 70)
    // openCV的hsv与标准HSV范围不同，做了如下的变换
    //    h= H/2;    s = (float)S/100*255;    v = (float)v/100*255;
    //色相,饱和度,亮度（绿色）
    Scalar hsvMin_G = Scalar(50, 70, 30);
    Scalar hsvMax_G = Scalar(75, 180, 200);

    Mat hsvImage;
    // bgr转hsv
    cvtColor(colorImg, hsvImage, CV_BGR2HSV);

    Mat greenMask;
    inRange(hsvImage, hsvMin_G, hsvMax_G, greenMask);
    imshow("inRange", greenMask);

    //形态学运算
    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(greenMask, greenMask, MORPH_OPEN, element); //开运算(去除小白点)
    element = getStructuringElement(MORPH_RECT, Size(11, 11));
    morphologyEx(greenMask, greenMask, MORPH_CLOSE, element); // 闭运算(去除黑点,连接一些连通域)
    imshow("greenMask", greenMask);

    vector<vector<Point>> contours; // 找到的各个轮廓的点,二维
    vector<Vec4i> hierarchy; // 轮廓层次结构
    // 寻找轮廓
    // 参数：二值图,找到的轮廓点,轮廓层次结构，轮廓的检索模式，轮廓估计的方法，偏移
    findContours(greenMask, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, Point(0, 0));
    // 面积最大的轮廓contours[0]
    sort(contours.begin(), contours.end(),
         [](vector<Point> &c1, vector<Point> &c2) { return contourArea(c1) > contourArea(c2); }); // 降序

    // 仅调试显示
    Mat contoursImg = Mat::zeros(colorImg.size(), CV_8UC3);
    Scalar color = Scalar(255, 255, 255); //任意值
    drawContours(contoursImg, contours, 0, color, 1, 8, hierarchy, 0, Point());
    imshow("contoursImg", contoursImg);

    //    //拟合多边形
    //    vector<vector<Point>> contours_poly(contours.size()); // 存储拟合多边形点集
    //    approxPolyDP(contours[0], contours_poly[0], 18, true);
    //    //绘制拟合后的多边形
    //    drawContours(contoursImg, contours_poly, 0, color, 1, 8);
    //    imshow("contoursImg", contoursImg);

    //    // 计算轮廓矩
    //    Moments mu = moments(contours[0], false);
    //    //  计算轮廓中心矩
    //    Point innerPoint = Point(mu.m10 / mu.m00, mu.m01 / mu.m00);
    //    // 点是否在轮廓内,多边形测试
    //    double cntDistance = 0;
    //    while (cntDistance <= 0) {
    //        cntDistance = pointPolygonTest(contours[0], innerPoint, true);
    //        innerPoint += Point(5, 5);
    //        cout << "-.-" << endl;
    //    }
    //    // todo 漫水填充？？有bug
    //    Mat fillImg = colorImg.clone();
    //    Rect ccomp;
    //    floodFill(fillImg, innerPoint, Scalar(0, 0, 0), &ccomp, Scalar(5, 5, 5), Scalar(8, 8, 8));
    //    imshow("fillImg", fillImg);

    outMask = Mat::zeros(colorImg.size(), CV_8UC1);
    // 填充轮廓内部 -1
    drawContours(outMask, contours, 0, Scalar(255), -1, 8, hierarchy, 0, Point());
    imshow("outMask", outMask);

    Mat greenBack;
    cv::bitwise_and(colorImg, colorImg, greenBack, outMask);
    imshow("greenBack", greenBack);

    imshow("colorImg", colorImg);
}

bool Detection::isExistLine() {
    return isHasLine;
}
