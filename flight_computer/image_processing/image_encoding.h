#pragma once
#include <array>
#include "color.h"
#include <memory>
#include <vector>
#include <cstring>

namespace image_processing{

#pragma pack(push, 1)
struct image_encoding_header {
    int height;
    int width;
};

struct image_encoding{
    image_encoding_header header;
    std::vector<color> pixels;
    
    image_encoding(){
        header.height = 0;
        header.width = 0;
    }
    
    image_encoding(int height_ , int width_) {
        header.height = height_;
        header.width = width_;
        pixels.resize(width_ * height_);
    }
    
    image_encoding(uint8_t* data, int size){
        insert_raw_data(data, size);
    }
    
    void insert_raw_data(uint8_t* data, int size){
        if(size < 8) {
            header.height = 0;
            header.width = 0;
            pixels.clear();
            return;
        }
        
        int new_height, new_width;
        std::memcpy(&new_height, data, sizeof(int));
        std::memcpy(&new_width, data + sizeof(int), sizeof(int));
        
        if(new_height < 0 || new_width < 0) {
            header.height = 0;
            header.width = 0;
            pixels.clear();
            return;
        }
        
        int pixel_size = new_height * new_width * sizeof(color);
        int total_size = 8 + pixel_size;
        
        if(size < total_size) {
            header.height = 0;
            header.width = 0;
            pixels.clear();
            return;
        }
        
        if(new_height != header.height || new_width != header.width) {
            header.height = new_height;
            header.width = new_width;
            pixels.resize(header.height * header.width);
        }
        
        std::memcpy(pixels.data(), data + 8, pixel_size);
    }
    
    uint8_t* raw_pixel_data(){
        return reinterpret_cast<uint8_t*>(pixels.data());
    }
    

   uint8_t* header_data(){
        return reinterpret_cast<uint8_t*>(&header);
    }
    
    size_t header_size(){
        return sizeof(image_encoding_header);
    }
    
    size_t pixel_data_size(){
        return header.height * header.width * sizeof(color);
    }
    
};
#pragma pack(pop)

}