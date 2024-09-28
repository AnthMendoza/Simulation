#ifndef VEHICLE_H
#define VEHICLE_H





class Vehicle{
    public:
        float   Xposition , Yposition , Zposition;     // position
        std::array<float,3> vehicleState;
        std::array<float,3> MOI;
        float mass;   
        float Xvelocity , Yvelocity , Zvelocity;
        float rollvelocity , pitchvelocity , yawvelocity;
        std::array<float,3> sumOfForces;
        std::array<float,3> sumOfMoments;

        Vehicle(float x, float y, float z, std::array<float, 3> mMOI , float mMass);

        void display();

        void drag();

        void addForce(std::array<float,3> forceVector);

        void updateState();

};



#endif