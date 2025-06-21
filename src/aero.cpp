#include "cmath"
#include "../include/aero.h"

namespace SimCore{

float airDensity(float Zposition) {  //Zposition in meters, returned in kg/m^3
    float temp = 0;
    float pressure = 0;

    if(Zposition < 11000){                                                      //Troposphere

        temp = 15.05f - .00649f *Zposition;
        pressure = 101.29f * pow((temp + 273.1f)/288.08f , 5.256);

    }else if(Zposition < 25000){                                                // lower Stratosphere

        temp = -56.46f;
        pressure = 22.65f * pow(2.7182818f , (1.73f - .000157f * Zposition));

    }else{                                                                      // upper Stratosphere

        temp = -131.21f + .00299f * Zposition;
        pressure = 2.488f * pow((temp + 273.1f)/216.6f , -11.388f);

    }

    return pressure / (.2869f * (temp + 273.15f));
}

float aeroAreaRocket(float angle){  //cross sectional aera dependent on the orientation of the vehicle. This will be estamited based on a cylindrical body
    // cross section of a circle when head-on, cross section of a rectangle when in slide slip.
    //coef of drag
    return sin(angle) * 141.687 + 10.75;
}


float coefOfDragRocket(float angle){
    //coef of drag for a "long" cylinder is .82 with blunt/square ends. 
    // for an infintly long cylinder, the sideways coef drag is 1.095
    return sin(angle) * .7 + .82;
}

float coefOfLiftRocket(float angle){
    // more is needed to properlly estmiate this
    return angle * 2.1;
}

//Overly Basic for testing. Should adapt to real drone profile
float aeroAreaDrone(float angle){
    return 1;
}

float coefOfDragDrone(float angle){
    return 0.5;
}
    
float coefOfLiftDrone(float angle){
    return 0.5;
}



}