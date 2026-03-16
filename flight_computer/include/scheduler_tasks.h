#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H
#include <functional>

namespace avionics{
    struct scheduler_tasks
    {
        const char* name;
        float interval_s;
        std::function<void(float)> callback;
    };
}

#endif