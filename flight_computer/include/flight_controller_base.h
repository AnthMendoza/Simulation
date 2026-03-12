#ifndef FLIGHT_CONTROLLER_BASE_H
#define FLIGHT_CONTROLLER_BASE_H
#include "avionics_states.h"
#include "control_tasks.h"

namespace avionics{
    class controller_base{
    private:

    protected:

        estimated_state vehicle_state;

        desired_state vehicle_desired_state;

    public:

        virtual std::vector<control_task> get_routine() = 0;

        void set_estimated_state(estimated_state state){
            vehicle_state = state;
        }

        estimated_state& get_estimated_state(){
            return vehicle_state;
        }

    };
}

#endif