#include "../telemetry/include/communication.h"
#include "../include/flight_computer.h"
#include "../pid_controller/include/pid_controller.h"




using namespace std;


int main(){

    //flight_computer::telemetry tel;
    //tel.start();
    avionics::flight_computer computer;
    avionics::pid::pid_config config;
    avionics::pid::controller_pid pid(config);
    computer.initialize(pid.get_routine());
    computer.start();
    string input;
    getline(cin,input);

    while(input != "exit"){
    }
    
    return 0;
}
