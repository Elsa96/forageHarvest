//
// Created by chao on 12/31/19.
//

#ifndef HARVEST_SRC_FRAME_HPP
#define HARVEST_SRC_FRAME_HPP

#include <opencv2/opencv.hpp>
#include <sl_zed/Camera.hpp>

// main 默认命令输入参数 ?? 全局的 未定义??
// extern char **argv;
// extern int argc;

class ImageFrame {
  public:
    ImageFrame();
    ~ImageFrame();

    // ! sl::Mat
    // GRBA还是BGRA,似乎是BGRA
    // * Create an  sl::Mat object
    // sl::Mat color_zed;
    sl::Mat color_zed; //
    // * Create an  sl::Mat object
    // sl::Mat depth_img_zed;
    sl::Mat depth_img_zed; //
    // * Create a sl::Mat with float type (32-bit)
    // sl::Mat depth_map_zed;
    sl::Mat depth_map_zed;
    // * Allocation of 4 channels of float XYZ[RGBA]
    // sl::Mat point_cloud_zed;
    sl::Mat point_cloud_zed; // XYZ[RGBA]
    // 分配空间位置可以不一致 默认GPU复制到CPU

    // ! cv::Mat
    // * color // CV_8UC4
    cv::Mat color_ocv;
    // * depth_img // CV_8UC4
    cv::Mat depth_img_ocv;
    // * depth_map // CV_32FC1
    cv::Mat depth_map_ocv;
    // * point_cloud_C4 // CV_32FC4 XYZ[RGBA]
    cv::Mat point_cloud_C4_ocv;
    // * point_cloud_C3 // CV_32FC3 XYZ
    cv::Mat point_cloud_C3_ocv;
    // * color_32FC1 // CV_32FC1 RGBA
    cv::Mat color_32FC1_ocv;

    unsigned long long timestamp;
    float current_fps;
    int index;
    int status;

    void splitColorChannel_C4toC3();
    //    bool ZED_grab_Img();
    bool ZED_grab_Img(sl::Camera &zed = zed_f, sl::RuntimeParameters &runtime_parameters = runtime_parameters_f);

    // * static
    //    static void setResolution(int width, int height);
    static void setResolutionScale(int scale);
    // Conversion function between sl::Mat and cv::Mat
    static cv::Mat slMat2cvMat(sl::Mat &input); // claim
    static bool camInit_zed(sl::Camera &zed = zed_f, sl::RuntimeParameters &runtime_parameters = runtime_parameters_f,
                            sl::InitParameters &init_params = init_params_f);
    //    static bool camInit_zed();
    static cv::Mat getIntrinsicMat();

    //    Create a ZED camera object
    static sl::Camera zed_f; // 可以有多个Camera对象

  private:
    // ! static 必须类外全局初始化,用了不初始化报未定义??
    // 私有权限 在类外不能访问,初始化无所谓private,public,都可以访问
    static int rel_width_f; // resolution = width * height
    static int rel_height_f;

    static cv::Mat intrinsic_f;
    // * Set runtime parameters after opening the camera
    static sl::RuntimeParameters runtime_parameters_f;
    // * Set configuration parameters for the ZED
    static sl::InitParameters init_params_f;
};

#endif // HARVEST_SRC_FRAME_HPP
