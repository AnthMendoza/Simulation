#ifndef FLIGHT_CONTROLLER_BASE_H
#define FLIGHT_CONTROLLER_BASE_H
#include "thread_manager.h"
#include "avionics_states.h"
#include "scheduler_tasks.h"

namespace avionics{
    class controller_base: public thread_manager{
    private:
        void initialize_base(){
           std::vector<scheduler_tasks> tasks = get_routine();
           for (auto& task : tasks){
               controller_scheduler.add_manager(task.interval_s,task.callback);
           }
           set_scheduler(controller_scheduler);
        }


    protected:

        scheduler controller_scheduler;

        estimated_state vehicle_state;

        desired_state vehicle_desired_state;

        virtual std::vector<scheduler_tasks> get_routine() = 0;

        virtual void initialize_implementation() = 0;


        void thread_startup_proccess() override{
            initialize_base();
            initialize_implementation();
        }

        void thread_proccess() override{
            controller_scheduler(start_to_recent_call_time());
        }

    public:




        void set_estimated_state(estimated_state state){
            vehicle_state = state;
        }

        estimated_state& get_estimated_state(){
            return vehicle_state;
        }


    };
}

#endif