#ifndef PID_CONFIG_LOADER_H
#define PID_CONFIG_LOADER_H

#include <yaml-cpp/yaml.h>
#include "pid_config.h"


namespace avionics{

namespace pid{


static p_con load_p_con(const YAML::Node& node);

static pid_con load_pid_con(const YAML::Node& node);

pid_config load_pid_config(const std::string& filepath);

}
}

#endif