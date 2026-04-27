#ifndef STATE_ESTIMATION_BASE_H
#define STATE_ESTIMATION_BASE_H
#include <util/thread_manager.h>
#include "../../include/scheduler_tasks.h"
#include <buffer_types.h>
#include <subsystem_monitor.h>

namespace avionics::estimation{

class estimation_base: public thread_manager , subsystem<&flight_computer_health::estimator>{

    private:
    scheduler estimator_scheduler;

    

    protected:

    state_dbuf& state_buffer;
    estimation_state_packet state_packet;

    sensor_dbuf& sensor_buffer;
    flight_health& health_buffer;
    
    void initialize_base(){
        std::vector<scheduler_tasks> tasks = get_routine();
        for (auto& task : tasks){
            estimator_scheduler.add_manager(task.interval_s,task.callback);
        }
        set_scheduler(estimator_scheduler);
    }

    void thread_process() override{
        estimator_scheduler(start_to_recent_call_time());
    }

    void thread_startup_process() override{
        initialize_base();
        SPDLOG_LOGGER_INFO(logger,"Estimation started.");
    }


    virtual std::vector<avionics::scheduler_tasks> get_routine() = 0;


    public:

    estimation_base(flight_health& health_buff, sensor_dbuf& sensor_buff,state_dbuf& state_buff ): health_buffer(health_buff), sensor_buffer(sensor_buff) ,state_buffer(state_buff) , subsystem<&flight_computer_health::estimator>(health_buff){

    }

};

}


#endif