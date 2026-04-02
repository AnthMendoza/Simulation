#include <dynamics/droneMinimal.h>
#include <subsystems/battery.h>
#include <subsystems/propeller.h>
#include <subsystems/motor.h>
#include <subsystems/droneDependencyInjector.h>
#include <sim_to_flight.h>
#include <actual_state.h>
#include <sensor_field.h>
#include <flight_computer.h>
#include <pid_controller.h>
#include <state_estimation_bypass.h>
#include <communication.h>




int main(){
    
    // --------------- drone Simulation ----------------

    SimCore::drone drone_m;

    std::string mission_path = std::string(HOVER_DIR);

    auto config_battery = readFileAsString(mission_path + "/configs/battery.toml");
    drone_m.set_battery<SimCore::battery>(config_battery);

    auto config_drone = readFileAsString(mission_path + "/configs/drone.toml");
    drone_m.initDrone(config_drone);

    auto config_propeller = readFileAsString(mission_path + "/configs/propeller.toml");
    auto config_motor = readFileAsString(mission_path + "/configs/motor.toml");

    SimCore::propeller prop(config_propeller);
    SimCore::motor mot(config_motor);

    auto motor_prop = SimCore::setSquare(0.3f,0.3f,prop,mot);
    drone_m.addMotorsAndProps(motor_prop);

    // --------------- Flight Computer ----------------
    
    avionics::flight_computer computer;
    avionics::pid::pid_config config;

    computer.set_controller<avionics::pid::controller_pid>(config);
    computer.set_telemetry<avionics::telemetry>();
    computer.set_estimator<avionics::estimation::estimation_bypass>();

    // --------------- Sim To Flight ----------------

    avionics::sim_to_flight<avionics::sensor::SensorField,avionics::sensor::actual_state> bridge;
    
    drone_m.setSimPublish(bridge);
    computer.setReadHardware(bridge);

    return 0;
}