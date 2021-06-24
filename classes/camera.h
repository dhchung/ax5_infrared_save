#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <string>
#include <opencv2/opencv.hpp>
#include <vector>

class Camera{
public:
    Camera(float min_temp_, float max_temp);
    ~Camera();

    Spinnaker::SystemPtr system;
    void print_spinnaker_version();
    bool camera_ready;

    cv::Mat acquire_image();
    void set_camera();

    float min_temp;
    float max_temp;

    float cal_ratio;
    float cal_bias;

    int min_pixel;
    int max_pixel;


private:

    std::string cam_serial;
    Spinnaker::CameraPtr camptr;
    Spinnaker::CameraList camList;

};