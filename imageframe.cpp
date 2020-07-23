//
// Created by chao on 12/31/19.
//

#include "imageframe.hpp"
#include "caminit_zed.hpp"
#include <iostream>

using namespace std;

// ! static 必须类外全局初始化,用了不初始化报未定义??
int ImageFrame::rel_width_f = 0;
int ImageFrame::rel_height_f = 0;
cv::Mat ImageFrame::intrinsic_f;

sl::Camera ImageFrame::zed_f; // 可以有多个Camera对象
// * Set runtime parameters after opening the camera
sl::RuntimeParameters ImageFrame::runtime_parameters_f;
// * Set configuration parameters for the ZED
sl::InitParameters ImageFrame::init_params_f;

// ! 不知道为啥必须放到初始化表里,不然point_cloud_zed报错,其他也不正常
ImageFrame::ImageFrame()
    : color_zed(sl::Mat(rel_width_f, rel_height_f, sl::MAT_TYPE_8U_C4, sl::MEM_CPU)),
      depth_img_zed(sl::Mat(rel_width_f, rel_height_f, sl::MAT_TYPE_8U_C4, sl::MEM_CPU)),
      depth_map_zed(sl::Mat(rel_width_f, rel_height_f, sl::MAT_TYPE_32F_C1, sl::MEM_CPU)),
      point_cloud_zed(sl::Mat(rel_width_f, rel_height_f, sl::MAT_TYPE_32F_C4, sl::MEM_CPU)) {
    //    cout << "ZED rel_width_f: " << rel_width_f << " , rel_height_f: " << rel_height_f << endl; //
    //    在构造之前修改值有效

    // 类组合的构造方式?? // 结构体方式必须立即初始化,同时指定构造数据格式类型
    // ! sl::Mat  RGBA还是BGRA,似乎是BGRA
    // * Create an  sl::Mat object
    // sl::Mat color_zed;
    //    color_zed = sl::Mat(rel_width_f, rel_height_f, sl::MAT_TYPE_8U_C4, sl::MEM_CPU); //
    //        sl::Mat(new_width, new_height, sl::MAT_TYPE_8U_C4, sl::MEM_CPU), // 指定图片大小
    // * Create an  sl::Mat object
    // sl::Mat depth_img_zed;
    //    depth_img_zed = sl::Mat(rel_width_f, rel_height_f, sl::MAT_TYPE_8U_C4, sl::MEM_CPU); //
    // * Create a sl::Mat with float type (32-bit)
    // sl::Mat depth_map_zed;
    //    depth_map_zed = sl::Mat(rel_width_f, rel_height_f, sl::MAT_TYPE_32F_C1, sl::MEM_CPU);
    // * Allocation of 4 channels of float XYZ[RGBA]
    // sl::Mat point_cloud_zed;
    //    point_cloud_zed = sl::Mat(rel_width_f, rel_height_f, sl::MAT_TYPE_32F_C4, sl::MEM_CPU); // XYZ[RGBA]
    // ! 点云图很特殊,不知道为啥point_cloud_zed必须放到初始化表里, 不然报错
    // 而且point_cloud_zed用结构体方式也必须立即初始化,同时指定构造数据格式类型
    // 分配空间位置可以不一致默认GPU复制到CPU

    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
    // 用ZED sl::Mat的地址初始化cv::Mat,使共享数据内存
    // 初始化时用zed的地址构造,也可以先默认构造再赋值
    // ! OpenCV::Mat shares sl::Mat data
    // * To share data between sl::Mat and cv::Mat, use 自定义 slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
    // The cv::Mat is automatically updated
    // * color_ocv // CV_8UC4
    color_ocv = slMat2cvMat(color_zed);
    // * depth_img_ocv // CV_8UC4
    depth_img_ocv = slMat2cvMat(depth_img_zed);
    // * depth_map_ocv // CV_32FC1
    depth_map_ocv = slMat2cvMat(depth_map_zed);
    // * point_cloud_C4_ocv // CV_32FC4
    point_cloud_C4_ocv = slMat2cvMat(point_cloud_zed); // CV_32FC4 XYZ[RGBA]
    // ! 读到的是CV_32FC4 需 CV_32FC3
    // * point_cloud_C3_ocv  // CV_32FC3 // 已经默认构造了,只是为了说明数据格式
    point_cloud_C3_ocv = cv::Mat(point_cloud_C4_ocv.size(), CV_32FC3); // CV_32FC3 XYZ
    // * color_32FC1_ocv // CV_32FC1 RGBA
    color_32FC1_ocv = cv::Mat(point_cloud_C4_ocv.size(), CV_32FC1); // CV_32FC1 RGBA

    cout << "-------------------------ImageFrame constructor--------------------------------" << endl;
    cout << "# color_zed # DataType: " << color_zed.getDataType() << endl;
    cout << "Infos: " << color_zed.getInfos() << endl;
    cout << "# depth_img_zed # DataType: " << depth_img_zed.getDataType() << endl;
    cout << "Infos: " << depth_img_zed.getInfos() << endl;
    cout << "# depth_map_zed # DataType: " << depth_map_zed.getDataType() << endl;
    cout << "Infos: " << depth_map_zed.getInfos() << endl;
    cout << "# point_cloud_zed # DataType: " << point_cloud_zed.getDataType() << endl;
    cout << "Infos: " << point_cloud_zed.getInfos() << endl;
}

ImageFrame::~ImageFrame() {
    color_zed.free(); //
    depth_img_zed.free(); //
    depth_map_zed.free();
    point_cloud_zed.free(); // XYZ
}

// void ImageFrame::setResolution(int width, int height) {
//    rel_width_f = width; // resolution = width * height
//    rel_height_f = height;
//}

void ImageFrame::setResolutionScale(int scale) {
    // * Prepare new image size to retrieve half-resolution images
    sl::Resolution image_size = zed_f.getResolution();
    rel_width_f = image_size.width / scale; // resolution = width * height
    rel_height_f = image_size.height / scale;
}

/**
 * Conversion function from sl::Mat to cv::Mat
 * @param input : sl::Mat
 * @return cv::Mat : cv::Mat
 **/
cv::Mat ImageFrame::slMat2cvMat(sl::Mat &input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
    case sl::MAT_TYPE_32F_C1:
        cv_type = CV_32FC1; // 5
        break;
    case sl::MAT_TYPE_32F_C2:
        cv_type = CV_32FC2; // 13
        break;
    case sl::MAT_TYPE_32F_C3:
        cv_type = CV_32FC3; // 21
        break;
    case sl::MAT_TYPE_32F_C4:
        cv_type = CV_32FC4; // 29
        break;
    case sl::MAT_TYPE_8U_C1:
        cv_type = CV_8UC1; // 0
        break;
    case sl::MAT_TYPE_8U_C2:
        cv_type = CV_8UC2; // 8
        break;
    case sl::MAT_TYPE_8U_C3:
        cv_type = CV_8UC3; // 16
        break;
    case sl::MAT_TYPE_8U_C4:
        cv_type = CV_8UC4; // 24
        break;
    default:
        break;
    }
    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // * cv::Mat and sl::Mat will share a single memory structure
    // getPtr() // Returns the CPU or GPU data pointer
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}

/**
 * separate the point color XYZ[RGBA] into XYZ, from cv_32FC4 to cv_32FC3 + cv_32FC1
 * @param in_pointCloudImg_cv32FC4
 * @param out_pointCloudVal_cv32FC3
 * @param out_color_cv32FC1
 **/
void ImageFrame::splitColorChannel_C4toC3() {
    // cout << "----------- splitColorChannel_C4toC3 --------------" << endl;
    cv::Mat &in_mixChan = point_cloud_C4_ocv; // mix输入Mat CV_32FC4
    cv::Mat &out_pointCloudVal_32FC3 = point_cloud_C3_ocv; // CV_32FC3 XYZ 分离的点云数据3通道
    cv::Mat &out_separateColor_32FC1 = color_32FC1_ocv; // CV_32FC1 [RGBA] 分离的颜色1通道
    // Mat out_pointCloudVal_32FC3(cloudImg.rows, cloudImg.cols, CV_32FC3); // CV_32FC3 XYZ 分离的点云数据3通道
    // Mat out_separateColor_32FC1(cloudImg.rows, cloudImg.cols, CV_32FC1); // CV_32FC1 [RGBA] 分离的颜色1通道
    cv::Mat out_mixChan[] = {out_pointCloudVal_32FC3, out_separateColor_32FC1}; // mix输出Mat数组
    // rgba[0] -> bgr[0], rgba[1] -> bgr[1], rgba[2] -> bgr[2], rgba[3] -> alpha[3]
    int from_to[] = {0, 0, 1, 1, 2, 2, 3, 3}; // 两个一组，通道复制对应关系，0->0,1->1,2->2,3->3
    mixChannels(&in_mixChan, 1, out_mixChan, 2, from_to, 4); // ! 复制前3个通道到pointCloud, 第4通道到color
}

// grab image from camera and store to CVImgFrame
// bool ImageFrame::ZED_grab_Img() {
bool ImageFrame::ZED_grab_Img(sl::Camera &zed, sl::RuntimeParameters &runtime_parameters) {

    // ! Grab an frame from camera
    cout << "--- grab frame --- " << endl;
    if (zed.grab(runtime_parameters) == sl::SUCCESS) { // A new image is available if grab() returns SUCCESS
        // ! time stamp
        // TIME_REFERENCE_IMAGE 以每一帧为参考grub后retrieveImage的时间戳都相同
        // TIME_REFERENCE_CURRENT 以当前时间为参考，返回当前时间戳
        timestamp = zed.getTimestamp(sl::TIME_REFERENCE_IMAGE); // timeStamp
        cout << "timestamp: " << timestamp << endl;
        // ! Current FPS
        current_fps = zed.getCurrentFPS();
        cout << "FPS: " << setprecision(3) << setiosflags(std::ios::showpoint) << current_fps << endl;

        // ! Retrieve left image
        // ! 此处可直接传到对应申请的空间数组内，而不需要下面的深拷贝
        zed.retrieveImage(color_zed, sl::VIEW_LEFT, sl::MEM_CPU, rel_width_f, rel_height_f); // Get the left image
        // sl::VIEW_LEFT, Left RGBA image. Each pixel contains 4 usigned char (R,G,B,A). sl::MAT_TYPE_8U_C4.
        //        zed.retrieveImage(imgFrame_zed.color_zed, sl::VIEW_LEFT, sl::MEM_CPU,new_width,new_height); //设置大小
        //        cout << "# color_zed # DataType: " << imgFrame.color_zed.getDataType() << endl;
        //        cout << "Infos: " << imgFrame.color_zed.getInfos() << endl;
        // * Display image with OpenCV
        // Display the left image from the cv::Mat object which share sl:Mat data
        //        cv::imshow("imgFrame.color_ocv", color_ocv); //必须waitKey才能imshow正常显示，其他延时都不行
        //        cv::waitKey(1);

        // ! Retrieve depth image. Depth is aligned on the left image [RGBA]
        zed.retrieveImage(depth_img_zed, sl::VIEW_DEPTH, sl::MEM_CPU, rel_width_f,
                          rel_height_f); // Retrieve the normalized depth image
        // By default, images are copied from GPU memory to CPU memory (RAM) when this function is called.
        // sl::MEM_CPU avoiding this copy.
        // Display the depth view from the cv::Mat object which share sl:Mat data
        //        cv::imshow("Depth", imgFrame.depth_img_ocv); // !!! 必须waitKey才能imshow正常显示，其他延时都不行
        //        cv::waitKey(1);// !!! 必须waitKey才能imshow正常显示

        // ! Retrieve depth map. Depth is aligned on the left image [RGBA]
        // The depth matrix stores 32-bit floating-point values which represent depth (Z) for each (X,Y) pixel.
        zed.retrieveMeasure(depth_map_zed, sl::MEASURE_DEPTH, sl::MEM_CPU, rel_width_f,
                            rel_height_f); // Retrieve depth map 32bit-float
        //        cout << "# depth_map_zed # DataType: " << imgFrame.depth_map_zed.getDataType() << endl;
        //        cout << "Infos: " << imgFrame.depth_map_zed.getInfos() << endl;
        // float depth_value = 0;
        // depth_map_zed.getValue(x, y, &depth_value); // Get the depth values for pixel (i,j)

        // ! Retrieve colored point cloud. Point cloud is aligned on the left image.
        zed.retrieveMeasure(point_cloud_zed, sl::MEASURE_XYZRGBA, sl::MEM_CPU, rel_width_f,
                            rel_height_f); // 4*32bit-float XYZ[RGBA]
        //  sl::MEASURE_XYZRGBA  sl::MEASURE_XYZBGRA sl::MEASURE_XYZ 多种格式  都是4*32bit-float XYZ[RGBA]
        // zed.retrieveMeasure(point_cloud_zed, sl::MEASURE_XYZRGBA, sl::MEM_GPU); // 4*32bit-float XYZ[RGBA]
        // ? adjusting depth resolution to speed up ?? glviewer could not display？？
        // zed.retrieveMeasure(point_cloud_zed, sl::MEASURE_XYZRGBA, sl::MEM_CPU, new_width, new_height); // 设置大小
        //        cout << "# point_cloud_zed # DataType: " << imgFrame.point_cloud_zed.getDataType() << endl;
        //        cout << "Infos: " << imgFrame.point_cloud_zed.getInfos() << endl;
        // COPY_TYPE?? //GPU CPU 之间传数据

        // * sl::point clout value
        // sl::float4 point_cloud_value;
        // point_cloud_zed.getValue(x, y, &point_cloud_value); // Get the 3D point cloud values for pixel (i,j)
        // float x = point_cloud_value.x  // float y = point_cloud_value.y  // float z =  point_cloud_value.z
        // float color = point_cloud_value.w; // 1*32bit=4*8bit [RGBA]

        // 非必须
        // !!! cv32FC4 -> cv32FC3 分离最后一个颜色通道
        splitColorChannel_C4toC3();

        return true;
    } else {
        cout << "!!! Grab an image failed !!!" << endl;
        sl::sleep_ms(1);

        return false;
    }
}

// bool ImageFrame::camInit_zed() {
bool ImageFrame::camInit_zed(sl::Camera &zed, sl::RuntimeParameters &runtime_parameters,
                             sl::InitParameters &init_params) {
    // * Set runtime parameters after opening the camera
    //    sl::RuntimeParameters runtime_parameters; // 定义变量
    //    runtime_parameters.sensing_mode = sl::SENSING_MODE_STANDARD; // Use STANDARD sensing mode
    runtime_parameters.sensing_mode = sl::SENSING_MODE_FILL; // ! 填充空点

    // * Set configuration parameters for the ZED
    //    sl::InitParameters init_params;// 定义变量
    init_params.camera_resolution = sl::RESOLUTION_HD720; // resolution HD720 1280*720
    init_params.camera_fps = 15; // Set fps at 30
    //    init_params.depth_mode = sl::DEPTH_MODE_ULTRA; // Use ULTRA depth mode  最高精度
    init_params.depth_mode = sl::DEPTH_MODE_QUALITY; // Use PERFORMANCE depth mode
    //    init_params.depth_mode = sl::DEPTH_MODE_PERFORMANCE; // Use PERFORMANCE depth mode 最高速度
    init_params.coordinate_units = sl::UNIT_MILLIMETER; // Set units in millimeter
    // init_params.coordinate_units =sl::UNIT_METER;
    //    init_params.depth_minimum_distance = 3000; // Set the minimum depth perception distance to 300mm
    //    测量的最小深度值 ,必须30cm--3m
    //    zed.setDepthMaxRangeValue(600); // Set the maximum depth perception distance to
    //    12000mm 设置最远测量范围,不影响性能，只影响点云值？？
    init_params.coordinate_system =
        sl::COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed
    // ! write runtime log to file
    // init_params.sdk_verbose = true; // default Disable verbose mode
    // init_params.sdk_verbose_log_file = "./runtime.log"; // redirect SDK verbose messages and console to a file.

    // * open SVO if one given as parameter
    //    if (argc > 1 && string(argv[1]).find(".svo"))
    //        init_params.svo_input_filename = argv[1];

    // * Open the camera
    sl::ERROR_CODE zed_error = zed.open(init_params);
    if (zed_error != sl::SUCCESS) {
        cout << "!!! ZED open failed !!!" << zed_error << endl;
        zed.close();
        exit(1);
        return false; // Quit if an error occurred
    }

    // 设置图片默认缩放比例
    setResolutionScale(1);

    // * Camera Information
    printf("ZED Model                 : %s\n", toString(zed.getCameraInformation().camera_model).c_str());
    printf("ZED Serial Number         : %d\n", zed.getCameraInformation().serial_number);
    printf("ZED Firmware              : %d\n", zed.getCameraInformation().firmware_version);
    printf("ZED Camera Resolution     : %dx%d\n", (int)zed.getResolution().width, (int)zed.getResolution().height);
    printf("ZED Camera FPS            : %d\n", (int)zed.getCameraFPS());

    return true;
}

cv::Mat ImageFrame::getIntrinsicMat() {
    // * Camera intrinsic parameters // 内参 定值
    float focal_x = ImageFrame::zed_f.getCameraInformation()
                        .calibration_parameters.left_cam.fx; // Focal length in pixels along x axis.
    float focal_y = ImageFrame::zed_f.getCameraInformation()
                        .calibration_parameters.left_cam.fy; // Focal length in pixels along y axis.
    float center_x = ImageFrame::zed_f.getCameraInformation()
                         .calibration_parameters.left_cam.cx; // Optical center along x axis, defined in pixels
    float center_y = ImageFrame::zed_f.getCameraInformation()
                         .calibration_parameters.left_cam.cy; // Optical center along y axis, defined in pixels
    // ! update camera intrinsic parameters
    intrinsic_f = (cv::Mat_<float>(3, 3) << focal_x, 0.000000, center_x, 0.000000, focal_y, center_y, 0.0, 0.0, 1.0);
    //    cout << "intrinsic =" << endl << intrinsic << endl;

    return intrinsic_f;
}
