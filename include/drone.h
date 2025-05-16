#ifndef DRONE_H
#define DRONE_H
#pragma once
#include "vehicle.h"

class drone :  public Vehicle{

    public:
    drone();
    void init() override;

};




#endif