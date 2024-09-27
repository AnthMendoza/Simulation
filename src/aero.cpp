#include "cmath"
#include "../include/aero.h"



float airDensity(float Zposition) {
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



float aeroArea(float angle){  //cross sectional aera dependent on the orientation of the vehicle. This will be estamited based on a cylindrical body
    // cross section of a circle when head on transforms into the cross section of a rectangle when in slide slip.
    //coef of drag
    return sin(angle) * 141.687 + 10.75;
    
}


float coefOfDrag(float angle){
    //coef of drag for a "long" cylinder is .82 with blunt/square ends. 
    // for an infintly long cylinder, the sideways coef drag is 1.095
    return sin(angle) * .38 + .82;

}
