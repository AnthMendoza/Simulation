#ifndef DRONE_H
#define DRONE_H
#pragma once
#include "vehicle.h"
namespace SimCore{
class drone :  public Vehicle{
    private:
    void motorThrust();
    protected:

    public:
    drone();
    void init() override;

};

}


#endif