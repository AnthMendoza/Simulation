#include "../include/MySim.h"
#include <iostream>

int main(){
    const char* path = "../configs/Rocket_Confi.toml";
    SimCore::unreal real(path);
    for(float i = 0 ; i < 10 ; i++){
        SimCore::unrealData packet  = *real.simFrameRequest(i);
        std::cout<< "its working"<< packet.position[2];
    }


    return 0;
}