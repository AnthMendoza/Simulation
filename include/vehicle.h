#ifndef VEHICLE_H
#define VEHICLE_H





class Vehicle{
    public:
        float   Xposition , Yposition , Zposition;     // position
        float    roll , pitch, yaw ;    // rotation
        float MOI[3], mass;   
        float Xvelocity , Yvelocity , Zvelocity;
        float rollvelocity , pitchvelocity , yawvelocity;
        float sumOfForces[3] , sumOfMoments[3];

        Vehicle(float x, float y, float z, float mroll, float mpitch, float myaw, float mMOI[3] , float mMass);

        void display();

        void includeForce();

        float drag();

        void addForce(float stateVector[3] , float forceIncident , float forceVector[3]);

        void updateState();

};



#endif