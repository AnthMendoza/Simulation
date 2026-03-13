#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "../../include/flight_controller_base.h"
#include "../../../simulation/include/control/PIDController.h"
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

            //used std::optional as a work around for PIDController() = delete

            std::array<std::optional<PIDController>,3> position_pid;
            std::array<std::optional<PIDController>,3> velocity_pid;

            struct subroutine_translation_layer{

                std::array<float,3> delta_position;
                std::array<float,3> req_velocity;
                std::array<float,3> delta_velocity;
                std::array<float,3> req_acceleration;
                std::array<float,3> delta_acceleration;

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



        public:

            controller_pid() = delete;
            controller_pid(pid_config& config);
            
            std::vector<control_task> get_routine() override; 

        };
    }
}

#endif