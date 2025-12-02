#include "../image_processing/display.h"



int main(){

    std::shared_ptr<image_processing::image_encoding> encoded = std::make_shared<image_processing::image_encoding>(100,100);

   auto& color = encoded->pixels[10];
   color.R = std::numeric_limits<uint8_t>::max();  

    display_image display(encoded);

    display.start_display();

    while(true){
        
    }

}