#ifndef PID_CONFIG_LOADER_H
#define PID_CONFIG_LOADER_H

#include <yaml-cpp/yaml.h>
#include "pid_config.h"


namespace avionics{

namespace pid{

pid_config load_pid_config(const std::string& filepath);

}
}

#endif