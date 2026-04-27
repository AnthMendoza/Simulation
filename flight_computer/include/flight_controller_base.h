#ifndef FLIGHT_CONTROLLER_BASE_H
#define FLIGHT_CONTROLLER_BASE_H
#include <base/base.h>
#include <util/thread_manager.h>
#include "avionics_states.h"
#include "scheduler_tasks.h"
#include <sensor_field.h>
#include <mutex>
#include <buffer_types.h>
#include <airframe_config.h>
#include <subsystem_monitor.h>

namespace avionics{
    class controller_base: public thread_manager , subsystem<&flight_health::controller> {
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
        
        flight_controller_telemetry_packet telemetry_packet;

        airframe_config configuration;

        desired desired_vehicle_state;

        telem_ring& telemetry_buffer;
        state_dbuf& state_buffer;
        nav_ring& navigation_buffer;

        virtual std::vector<scheduler_tasks> get_routine() = 0;
        virtual void initialize_implementation() = 0;
        virtual void set_telemetry(float time) = 0;

        virtual desired get_navigation(){
            auto new_desired =  navigation_buffer.pop_latest();
            if(new_desired == std::nullopt) return desired_vehicle_state;
            desired_vehicle_state = new_desired.value();
            return desired_vehicle_state;
        }   

        std::function<void(float)> set_telemetry_callback;

        void set_telemetry_buffer(std::function<void(float)> callback){
            set_telemetry_callback = callback;
        }

        

        void thread_startup_process() override{
            initialize_base();
            initialize_implementation();
            set_running();
        }

        void thread_process() override{
            controller_scheduler(start_to_recent_call_time());
        }


    public:

        controller_base(flight_health& health, state_dbuf& state_buff , telem_ring& tel_buff, nav_ring& nav_buff  ,const airframe_config& config) : 
            state_buffer(state_buff), telemetry_buffer(tel_buff) , navigation_buffer(nav_buff) , configuration(config), subsystem<&flight_health::controller>(health){

        }


        void set_estimated_state(estimated_state state){
            vehicle_state = state;
        }

        estimated_state& get_estimated_state(){
            return vehicle_state;
        }

        nav_ring& get_nav_buffer(){
            return navigation_buffer;
        }


    };
}

#endif