#pragma once 

#define DEV_MODE
#ifdef DEV_MODE

#include <opencv2/opencv.hpp>
#include "image_encoding.h"
#include <string>
#include <atomic>
#include <chrono>
#include <thread>

class display_image {
    private:

    std::weak_ptr<image_processing::image_encoding> image;
    std::string display_name;
    std::atomic_bool displaying;
    std::atomic<bool> display_on;

    public:
    display_image(std::shared_ptr<image_processing::image_encoding> shared_img , std::string name = "Display"): 
    image(shared_img) , display_name(name){
        
    }

    void start_display(){
        auto image_ptr = image.lock();
        if(image_ptr == nullptr){
            return;
        }

        cv::Mat img(image_ptr->header.height,image_ptr->header.width,CV_8UC3);
        img.data = image_ptr->raw_pixel_data();
        display_on = true;

        while(display_on){
            cv::imshow(display_name, img);
            cv::waitKey(1);
            std::this_thread::sleep_for(std::chrono::milliseconds(8));

        }
    }

    void end_display(){
        display_on = false;
    }


};



#endif