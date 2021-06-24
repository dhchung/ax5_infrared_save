#include "camera.h"


Camera::Camera(float min_temp_, float max_temp_): min_temp(min_temp_), max_temp(max_temp_){
    system = Spinnaker::System::GetInstance();
    cam_serial = "73400300";
    camera_ready = false;
    camptr = nullptr;

    cal_ratio = (max_temp - min_temp)/16384.0f;
    cal_bias = min_temp;

    min_pixel = int((min_temp + 273.15f)*25); 
    max_pixel = int((max_temp + 273.15f)*25); 

    set_camera();
}
Camera::~Camera(){

    if(camera_ready){
        camptr->EndAcquisition();
        camptr->DeInit();
    }
    camList.Clear();
}

void Camera::print_spinnaker_version(){

    const Spinnaker::LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
    std::cout << "Spinnaker library version: " << spinnakerLibraryVersion.major << "." << spinnakerLibraryVersion.minor
         << "." << spinnakerLibraryVersion.type << "." << spinnakerLibraryVersion.build << std::endl
         << std::endl;
}


void Camera::set_camera(){
    camList = system->GetCameras();
    const unsigned int numCameras = camList.GetSize();
    std::cout<<"Number of cameras detected: "<<numCameras<<std::endl;

    camptr = camList.GetBySerial(cam_serial);

    if(camptr.IsValid()){
        camptr->DeInit();
        camptr->Init();
        camptr->AcquisitionMode.SetValue(Spinnaker::AcquisitionMode_Continuous);
        Spinnaker::GenApi::INodeMap& cam_nodemap = camptr->GetNodeMap();
        Spinnaker::GenApi::CEnumerationPtr node_pixel_format = 
            cam_nodemap.GetNode("PixelFormat");

        if(!Spinnaker::GenApi::IsAvailable(node_pixel_format) ||
           !Spinnaker::GenApi::IsWritable(node_pixel_format)) {
               std::cout<<"Unable to set pixel format, mate"<<std::endl;
        } else {
            Spinnaker::GenApi::CEnumEntryPtr node_pixel_format_mono14 = 
                Spinnaker::GenApi::CEnumEntryPtr(node_pixel_format->GetEntryByName("Mono14"));
            if(!Spinnaker::GenApi::IsAvailable(node_pixel_format_mono14) ||
               !Spinnaker::GenApi::IsReadable(node_pixel_format_mono14)) {
                std::cout<<"Unable to set pixel format into mono14, mate"<<std::endl;            
            } else {
                node_pixel_format->SetIntValue(node_pixel_format_mono14->GetValue());
                std::cout<<"Pixel format is set to mono14, son"<<std::endl;
            }
        }

        Spinnaker::GenApi::CEnumerationPtr node_temp_linear =
            cam_nodemap.GetNode("TemperatureLinearResolution");
        if(!Spinnaker::GenApi::IsAvailable(node_temp_linear) ||
           !Spinnaker::GenApi::IsWritable(node_temp_linear)) {
               std::cout<<"Unable to set temperature resolution, mate"<<std::endl;
        } else {
            Spinnaker::GenApi::CEnumEntryPtr node_temp_linear_high = 
                Spinnaker::GenApi::CEnumEntryPtr(node_temp_linear->GetEntryByName("High"));
            if(!Spinnaker::GenApi::IsAvailable(node_temp_linear_high) ||
               !Spinnaker::GenApi::IsReadable(node_temp_linear_high)) {
                std::cout<<"Unable to set temperature resolution to high, mate"<<std::endl;            
            } else {
                node_temp_linear->SetIntValue(node_temp_linear_high->GetValue());
                std::cout<<"Temperature resolution set to high, son"<<std::endl;

            }
        }

        Spinnaker::GenApi::CEnumerationPtr node_bit_depth =
            cam_nodemap.GetNode("CMOSBitDepth");
        if(!Spinnaker::GenApi::IsAvailable(node_bit_depth) ||
           !Spinnaker::GenApi::IsWritable(node_bit_depth)) {
               std::cout<<"Unable to set CMOS bit depth, mate"<<std::endl;
        } else {
            Spinnaker::GenApi::CEnumEntryPtr node_bit_depth_14bit = 
                Spinnaker::GenApi::CEnumEntryPtr(node_bit_depth->GetEntryByName("bit14bit"));
            if(!Spinnaker::GenApi::IsAvailable(node_bit_depth_14bit) ||
               !Spinnaker::GenApi::IsReadable(node_bit_depth_14bit)) {
                std::cout<<"Unable to set CMOS bit depth to 14bit, mate"<<std::endl;            
            } else {
                node_bit_depth->SetIntValue(node_bit_depth_14bit->GetValue());
                std::cout<<"CMOS bit depth set to 14bit, son"<<std::endl;
            }
        }

        Spinnaker::GenApi::CEnumerationPtr node_temp_linear_mode =
            cam_nodemap.GetNode("TemperatureLinearMode");
        if(!Spinnaker::GenApi::IsAvailable(node_temp_linear_mode) ||
           !Spinnaker::GenApi::IsWritable(node_temp_linear_mode)) {
               std::cout<<"Unable to set temperature linear mode, mate"<<std::endl;
        } else {
            Spinnaker::GenApi::CEnumEntryPtr node_temp_linear_mode_on = 
                Spinnaker::GenApi::CEnumEntryPtr(node_temp_linear_mode->GetEntryByName("On"));
            if(!Spinnaker::GenApi::IsAvailable(node_temp_linear_mode_on) ||
               !Spinnaker::GenApi::IsReadable(node_temp_linear_mode_on)) {
                std::cout<<"Unable to set temperature linear mode to on, mate"<<std::endl;            
            } else {
                node_temp_linear_mode->SetIntValue(node_temp_linear_mode_on->GetValue());
                std::cout<<"Temperature linear mode is on, son"<<std::endl;
            }
        }

        Spinnaker::GenApi::INodeMap & sNodemap = camptr->GetTLStreamNodeMap();
        
        Spinnaker::GenApi::CEnumerationPtr node_bufferhandling_mode = 
            sNodemap.GetNode("StreamBufferHandlingMode");
        if(!Spinnaker::GenApi::IsAvailable(node_bufferhandling_mode) ||
           !Spinnaker::GenApi::IsWritable(node_bufferhandling_mode)) {
               std::cout<<"Unable to set buffer handling mode, mate"<<std::endl;
        } else {
            Spinnaker::GenApi::CEnumEntryPtr node_bufferhandling_mode_NewestOnly = 
                Spinnaker::GenApi::CEnumEntryPtr(node_bufferhandling_mode->GetEntryByName("NewestOnly"));
            if(!Spinnaker::GenApi::IsAvailable(node_bufferhandling_mode_NewestOnly) ||
               !Spinnaker::GenApi::IsReadable(node_bufferhandling_mode_NewestOnly)) {
                std::cout<<"Unable to set buffer handling mode to newest only, mate"<<std::endl;            
            } else {
                node_bufferhandling_mode->SetIntValue(node_bufferhandling_mode_NewestOnly->GetValue());
                std::cout<<"Buffer handling mode set to newest only, son"<<std::endl;
            }
        }            

        camptr->BeginAcquisition();
        camera_ready = true;
    } else {
        std::cout<<"Camera not ready"<<std::endl;
        camera_ready = false;
    }
}


cv::Mat Camera::acquire_image(){

    cv::Mat image;
    if(camera_ready){
        Spinnaker::ImagePtr img = camptr->GetNextImage();

        if(img->IsIncomplete()) {
            std::cout<<"Image Incomplete: "<<Spinnaker::Image::GetImageStatusDescription(img->GetImageStatus())<<std::endl;
            return image;
        } else {
            const size_t width = img->GetWidth();
            const size_t height = img->GetHeight();
            // std::cout<<width<<", "<<height<<std::endl;
            image = cv::Mat(cv::Size(width, height), CV_16UC1, img->GetData());
            std::cout<<"before :"<<(image.at<u_int16_t>(width/2, height/2))*0.04 - 273.15<<std::endl;

            for(int i = 0; i < image.rows; ++i) {
                for(int j = 0; j < image.cols; ++j) {
                    // Range : -273.15 to 382.17
                    int pixel_value = int(image.at<u_int16_t>(i,j));

                    if(pixel_value <= min_pixel) {
                        image.at<u_int16_t>(i,j) = 0;
                    } else if(pixel_value >= max_pixel) {
                        image.at<u_int16_t>(i,j) = 16383;
                    } else {
                        image.at<u_int16_t>(i,j) = int(float(image.at<u_int16_t>(i,j) - min_pixel) * (16383.0f/float(max_pixel - min_pixel)));
                    }
                    // float temp_value = float(pixel_value) * 0.04 - 273.15;

                    image.at<u_int16_t>(i,j) = image.at<u_int16_t>(i,j) * 4;
                }
            }


            std::cout<<"after :"<<float(image.at<u_int16_t>(width/2, height/2)/4)*cal_ratio + cal_bias<<std::endl;

            cv::imshow("Infrared", image);
            cv::waitKey(1);

        }
    }
    else {
        std::cout<<"Check the camera mate,"<<std::endl;
    }

    return image;
}