#include <array>
#include "../include/vehicle.h"

int main(){
    std::array<float,3> MOI = {.2,.2,.2};
    Vehicle rocket(0,0,0,MOI, 42000);
    rocket.Zvelocity = 100;
    rocket.drag();
}