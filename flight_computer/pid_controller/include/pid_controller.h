#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "../../include/flight_controller_base.h"
#include <util/util.h>
#include "pid_config.h"
#include <optional>

namespace avionics{
    namespace pid{
        class controller_pid : public controller_base{
        private:

            

            void position(float time);
            void velocity(float time);
            void acceleration(float time);
            void angle_of_attack(float time);
            void set_telemetry(float time) override;
            void get_hardware();

            //used std::optional as a work around for PIDController() = delete

            std::array<std::optional<PIDController>,3> position_pid;
            std::array<std::optional<PIDController>,3> velocity_pid;

            struct subroutine_translation_layer{

                fc_units::vec3 delta_position;
                fc_units::vec3 req_velocity;
                fc_units::vec3 delta_velocity;
                fc_units::vec3 req_acceleration;
                fc_units::vec3 delta_acceleration;

                subroutine_translation_layer(): delta_position({0.0f,0.0f,0.0f}),delta_velocity({0.0f,0.0f,0.0f}),
                                                req_velocity({0.0f,0.0f,0.0f}),req_acceleration({0.0f,0.0f,0.0f}){
                }

            }subroutine;

            struct subroutine_last_call_log{

                float position;
                float velocity;
                float acceleration;
                float angle_of_attack;

            }last_call_time;
        
        protected:

            std::vector<scheduler_tasks> get_routine() override; 

            void initialize_implementation() override;


        public:

            controller_pid() = delete;
            controller_pid(state_dbuf& state_buff, telem_ring& tel_buff , const pid_config& config);
            

        };
    }
}

#endif