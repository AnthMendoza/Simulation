#ifndef PID_CONFIG_H
#define PID_CONFIG_H

#include <array>

namespace avionics{
namespace pid{

struct pid_gains{
    
    float kp;
    float ki;
    float kd;

    float min;
    float max;

    pid_gains(): kp(0), ki(0) , kd(0) , min(0) , max(0) {

    }
};

struct pid_config{
    std::array<pid_gains,3> position;
    std::array<pid_gains,3> velocity;

};

}
}


#endif