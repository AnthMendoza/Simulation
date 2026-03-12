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

            void position();
            void velocity();
            void acceleration();
            void angle_of_attack();

            //used std::optional as a work around for PIDController() = delete

            std::array<std::optional<PIDController>,3> position_pid;
            std::array<std::optional<PIDController>,3> velocity_pid;

            struct subroutine_translation_layer{

                std::array<float,3> delta_position;
                std::array<float,3> delta_velocity;

                subroutine_translation_layer(): delta_position({0.0f,0.0f,0.0f}),delta_velocity({0.0f,0.0f,0.0f}){
                }
            }subroutine;

        public:

            controller_pid() = delete;
            controller_pid(pid_config& config);
            
            std::vector<control_task> get_routine() override; 

        };
    }
}

#endif