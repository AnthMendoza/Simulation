#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H
#include <functional>

namespace avionics{
    struct control_task
    {
        const char* name;
        float interval_s;
        std::function<void(float)> callback;
    };
}

#endif