#include <iostream>




class Vehicle{
    public:
        float   Xposition , Yposition , Zposition;     // position
        float    roll , pitch, yaw ;    // rotation

    Vehicle(float x, float y, float z, float mroll, float mpitch, float myaw){
        Xposition = x;
        Yposition = y;
        Zposition = z;
        roll = mroll;
        pitch = mpitch;
        yaw = myaw;

    }

    void display(){
         std::cout << "Position: (" << Xposition << ", " << Yposition << ", " << Zposition << ")\n"<< "Orientation (Roll, Pitch, Yaw): (" << roll << ", " << pitch << ", " << yaw << ")\n";
    }
};