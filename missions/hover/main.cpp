#include <dynamics/droneMinimal.h>
#include <battery.h>
#include <subsystems/propeller.h>
#include <subsystems/motor.h>
#include <subsystems/droneDependencyInjector.h>
#include <sim_to_flight.h>
#include <actual_state.h>
#include <sensor_field.h>
#include <flight_computer.h>
#include <pid_controller.h>
#include <estimation.h>
#include <communication.h>
#include <sim/hardware_sim.h>




int main(){
    
    // --------------- drone Simulation ----------------

    SimCore::drone drone_m;

    std::string mission_path = std::string(HOVER_DIR) + "/configs/";

    auto battery_path = mission_path + "battery.yaml";
    drone_m.set_battery<hardware::battery>(battery_path);

    auto config_drone = mission_path + "drone.yaml";
    drone_m.initDrone(config_drone);

    auto propeller_path = mission_path + "propeller.yaml";
    auto motor_path = mission_path + "motor.yaml";
    SimCore::propeller prop(propeller_path);
    SimCore::motor mot(motor_path);

    auto motor_prop = SimCore::setSquare(0.3f,0.3f,prop,mot);
    drone_m.addMotorsAndProps(motor_prop);

    // --------------- Flight Computer ----------------
    
    avionics::flight_computer computer(10, mission_path + "drone.yaml");

    auto config_telemetry_path = mission_path + "telemetry.toml";
    auto config_pid_path = mission_path + "controller.yaml";

    computer.set_controller<avionics::pid::controller_pid>(config_pid_path);
    computer.set_telemetry<avionics::telemetry>(config_telemetry_path);
    computer.set_estimator<avionics::estimation::estimation>();

    // --------------- Sim To Flight ----------------

    avionics::sim_to_flight<avionics::sensor::SensorField,avionics::sensor::actual_state> bridge;
    
    drone_m.setSimPublish(bridge);
    computer.set_hardware<avionics::sim::hardware_sim>(bridge);


    // --------------- Start ----------------

    computer.start();
    drone_m.start();
    
    while(true){

    }
    computer.stop();

    return 0;
}
