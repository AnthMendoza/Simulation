#include "../include/ground_station_startup.h"
#include <cstdlib>
#include <filesystem>
#include <iostream>

void ground_station::ground_station_startup::start(){

    open_python_gui();

}

void ground_station::ground_station_startup::end(){



}

void ground_station::ground_station_startup::open_python_gui() {

#if defined(__linux__) || defined(__APPLE__)

    if (!std::filesystem::exists(gui_path)) {
        std::cout << "File does not exist: " << gui_path << std::endl;
        return;
    }


    //the & sign is IMPORTANT. Ensures the gui is nonblocking and runs in the background
    std::string command = "python3 " + gui_path + "&";
    std::system(command.c_str());

#endif

}
