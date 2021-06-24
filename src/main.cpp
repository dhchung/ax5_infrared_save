#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>

#include "camera.h"
#include "parameters_json.h"

#include <thread>
#define PREFIX_PATH "/mnt/DataDisk/"

//Time Check
#include <chrono>
// Create directory
#include <sys/stat.h>
// Threading
#include <thread>

#define PREFIX_PATH "/mnt/DataDisk/"

Params params;

bool data_logging;
std::string data_prefix = "Stoplogging";
std::string stop_logging_msg = "Stoplogging";
std::string dir_name;

std::vector<std::thread> threads;
std::vector<bool> running_check;

std::vector<int> compression_params;

int data_no = 0;

FILE * ptr_time = nullptr;

void dataLoggingFlagCallback(const std_msgs::Bool::ConstPtr &msg){
    if(msg->data) {
        if(data_logging) {

        } else {
            data_logging = true;
            ROS_INFO("Data Logging Set True");
            data_no = 0;
        }
    } else {
        if(data_logging){
            data_logging = false;
            ROS_INFO("Data Logging Set False");

            for(int i = 0; i < threads.size(); ++i) {
                if(threads[i].joinable()) {
                    threads[i].join();
                }
            }

            threads.clear();
            running_check.clear();

        }
    }
}

void dataPrefixCallBack(const std_msgs::String::ConstPtr & msg) {
    if(msg->data.compare(stop_logging_msg)!=0) {
        if(data_prefix.compare(stop_logging_msg) == 0) {
            std::cout<<"Prefix changed to logging"<<std::endl;
            data_prefix = msg->data;
            dir_name = PREFIX_PATH + data_prefix + "_infrared";

            mkdir(dir_name.c_str(), 0777);

            std::string timestampPath = dir_name + "/timestamp.txt";
            ptr_time = fopen(timestampPath.c_str(), "a");
            if(!ptr_time)
                printf("FAILED to open %s\n", timestampPath.c_str()); 

        }
    } else {
        if(data_prefix.compare(stop_logging_msg)!=0) {
            std::cout<<"Prefix changed to stop logging"<<std::endl;
            data_prefix = msg->data;
            fclose(ptr_time);
        }
    }
}

void save_image(cv::Mat image, 
                std::string filename, 
                int thread_no) {
    running_check[thread_no] = true;
    cv::imwrite(std::string(filename), image);
    running_check[thread_no] = false;
}

int main(int argc, char ** argv) {
    std::string param_dir = "/home/morin/dev/catkin_ws/src/ax5_infrared_save/include/parameters.json";
    params.read_data(param_dir);

    data_no = 0;
    data_logging = false;

    Camera cam(params.thermalParam.min_temp, params.thermalParam.max_temp);

    ros::init(argc, argv, "ifcamera_send_image");
    ros::NodeHandle nh;

    cv_bridge::CvImage img_bridge;

    sensor_msgs::Image img;
    std_msgs::Header header;

    ros::Publisher pub_img = nh.advertise<sensor_msgs::Image>("infrared_cam/image", 1);

    ros::Subscriber sub_bool = nh.subscribe("/datalogging", 1, dataLoggingFlagCallback);
    ros::Subscriber sub_prefix = nh.subscribe("/save_prefix", 1, dataPrefixCallBack);

    ros::Rate loop_rate(20);
    int count = 0;
    int thread_num = 0;

    while(ros::ok()){

        cv::Mat acquired_image =  cam.acquire_image();
        // header.seq = count;
        // header.stamp = ros::Time::now();
        // img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16, acquired_image);
        // img_bridge.toImageMsg(img);

        // pub_img.publish(img);

        if(data_logging) {
            if(data_prefix.compare(stop_logging_msg)!=0) {
                char timestamp_buf[256];
                sprintf(timestamp_buf, "%06d\t%f\n", data_no, ros::Time::now().toSec());

                fwrite(timestamp_buf, 1, strlen(timestamp_buf), ptr_time);

                char filename[256];
                sprintf(filename, "%s/Image%06d_raw.png",dir_name.c_str(), data_no);

                bool all_thread_running = true;
                int empty_thread = -1;
                for(int i = 0; i < thread_num; ++i) {
                    if(running_check[i]){

                    } else{
                        empty_thread = i;
                        all_thread_running = false;
                        if(threads[i].joinable()){
                            threads[i].join();
                        }
                    }
                }

                if(all_thread_running) {
                    running_check.push_back(false);
                    threads.emplace_back(save_image, 
                                         acquired_image, 
                                         filename,
                                         running_check.size()-1);
                    threads[running_check.size()-1].detach();
                    ++thread_num;
                } else {
                    threads[empty_thread] = std::thread(save_image, 
                                                        acquired_image,
                                                        filename,
                                                        empty_thread);
                    threads[empty_thread].detach();    
                }

                int num_running_thread = 0;
                for(int i = 0; i < running_check.size(); ++i) {
                    if(running_check[i]) {
                        ++num_running_thread;
                    }
                }
                std::cout<<"running thread"<<num_running_thread<<std::endl;
                std::cout<<data_no<<std::endl;

                ++data_no;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
        // ++count;
    }
    return 0;
}