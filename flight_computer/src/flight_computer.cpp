#include "../telemetry/include/communication.h"



using namespace std;


int main(){

    flight_computer::telemetry tel;
    tel.start();
    string input;
    getline(cin,input);

    while(input != "exit"){
    }
    
    return 0;
}
