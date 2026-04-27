#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "../../include/flight_controller_base.h"
#include <util/util.h>
#include "pid_config.h"
#include <airframe_config.h>
#include <optional>

namespace avionics{
    class controlAllocator;
    namespace pid{
        class controller_pid : public controller_base{
        private:
            pid_config config_;
            

            void position(float time);
            void velocity(float time);
            void acceleration(float time);
            void angle_of_attack(float time);
            void set_telemetry(float time) override;
            void get_hardware();
            controller_feedback allocate();
            void parse_gains(const std::string path);

            //used std::optional as a work around for PIDController() = delete

            std::array<std::optional<PIDController>,3> position_pid;
            std::array<std::optional<PIDController>,3> velocity_pid;

            std::shared_ptr<controlAllocator> allocate_thrust;

            bool initialized_position = false;
            bool initialized_velocity = false;
            bool initialized_acceleration = false;
            bool initialized_aot = false;

            struct subroutine_translation_layer{

                fc_units::vec3 delta_position;
                fc_units::vec3 req_velocity;
                fc_units::vec3 delta_velocity;
                fc_units::vec3 req_acceleration;
                fc_units::vec3 delta_acceleration;

                subroutine_translation_layer(): delta_position({0.0f,0.0f,0.0f}),delta_velocity({0.0f,0.0f,0.0f}),
                                                req_velocity({0.0f,0.0f,0.0f}),req_acceleration({0.0f,0.0f,0.0f}){
                }

                void print(const std::shared_ptr<spdlog::logger>& logger) const {
                    SPDLOG_LOGGER_INFO(logger, "Subroutine Translation Layer:");

                    utility::print(logger, delta_position,"delta_position");
                    utility::print(logger,req_velocity,"req_velocity");
                    utility::print(logger,delta_velocity,"delta_velocity");
                    utility::print(logger, req_acceleration,"req_acceleration");
                    utility::print(logger, delta_acceleration,"delta_acceleration");
                }

            }subroutine;

            struct subroutine_last_call_log{
                float position = 0.f;
                float velocity = 0.f;
                float acceleration = 0.f;
                float angle_of_attack = 0.f;
            } last_call_time{};
        
        protected:

            std::vector<scheduler_tasks> get_routine() override; 

            void initialize_implementation() override;


        public:

            controller_pid() = delete;
            controller_pid(flight_health& health_buff, state_dbuf& state_buff, telem_ring& tel_buff , nav_ring& nav_buff
                            ,const airframe_config& air_config ,const std::string path);

            

        };
    }
}

#endif