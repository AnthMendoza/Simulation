#include "../telemetry/include/communication.h"
#include "../include/flight_computer.h"
#include "../pid_controller/include/pid_controller.h"
#include "../state_estimation/include/estimation.h"
#include "../sensor/include/actual_state.h"
#include <sensor_field.h>



using namespace std;


int main(){


    avionics::flight_computer computer;
    avionics::pid::pid_config config;

    

    //computer.set_controller<avionics::pid::controller_pid>(config);
    //computer.set_telemetry<avionics::telemetry>();
    //computer.set_estimator<avionics::estimation::estimation_>();

    //computer.start();
    string input;
    getline(cin,input);

    while(input != "exit"){
    }
    
    return 0;
}
