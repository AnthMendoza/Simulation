#ifndef VEHICLE_H
#define VEHICLE_H





class Vehicle{
    public:
        float   Xposition , Yposition , Zposition;     // position
        float vehicleState[3];
        float MOI[3], mass;   
        float Xvelocity , Yvelocity , Zvelocity;
        float rollvelocity , pitchvelocity , yawvelocity;
        float sumOfForces[3] , sumOfMoments[3];

        Vehicle(float x, float y, float z, float mMOI[3] , float mMass);

        void display();

        void drag();

        void addForce(std::array<float,3> forceVector);

        void updateState();

};



#endif