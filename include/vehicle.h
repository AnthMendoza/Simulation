#ifndef VEHICLE_H
#define VEHICLE_H





class Vehicle{
    public:
        float   Xposition , Yposition , Zposition;     // position
        float    roll , pitch, yaw ;    // rotation
        float MOI[3];   
        float Xvelocity , Yvelocity , Zvelocity;
        float rollvelocity , pitchvelocity , yawvelocity;

        Vehicle(float x, float y, float z, float mroll, float mpitch, float myaw, float mMOI[3]);

        void display();

        void updateState();

};



#endif