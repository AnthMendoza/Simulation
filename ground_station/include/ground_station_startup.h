#pragma once
#include <string>

namespace ground_station{

class ground_station_startup{

private:

std::string gui_path = "../gui/src/gui.py";

void open_python_gui();

public:

void start();

void end();

};

}