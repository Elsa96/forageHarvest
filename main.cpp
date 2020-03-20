#include <iostream>
#include "Detection.h"
#include "Cam.h"

int main() {
    Cam zedCamera;
    zedCamera.cameraStart();
    cv::Mat image = zedCamera.getImage(0);

    Detection d(image); //检测
    d.process();
    d.show();

    Point2f *vertex2D;
    vertex2D = d.getVertex(); //四个角点



    return 0;
}


