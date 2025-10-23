#ifndef ROCKET_H
#define ROCKET_H

#pragma once
#include "vehicle.h"
namespace SimCore{
class Rocket : public Vehicle{
    private:

    float finErrorX;
    float finErrorY;
    float finX;
    float finY;
    float sumOfFinErrorX;
    float sumOfFinErrorY;
    
    float finVelocityX;
    float finVelocityY;
    float finDamping;
    float finPGain;
    float finIGain;
    float finDGain;
    float maxFinAcceleration;


    protected:
    float gimbalErrorX;
    float gimbalErrorY;

    float sumOfGimbalErrorX;
    float sumOfGimbalErrorY;

    float gimbalVelocityX;
    float gimbalVelocityY;
    float gimbalDamping;
    float gimbalPGain;
    float gimbalIGain;
    float gimbalDGain;
    std::array<float ,2 > landingGimbalDirection;

    float landingRequiredThrust;
    float  PIDOutputY;
    float  PIDOutputX; 

    float maxGimbalAcceleration;
    float maxGimbalVelocity;

    float error;
    std::array<float,2> twoDAngle;
    
    //distance between the center of gravity to the center of pressure, this allows us to not have COG defined explicitly.
    //all forces will be in refrance to the Center of gravity 

    float cogToEngine;

    float liftAngleLog;

    bool reentry;

    bool glidePhase;

    bool landingInProgress;

    float vehicleYError;
    float sumOfVehicleYError;

    float vehicleXError;
    float sumOfVehicleXError;

    float maxGimbalAngle;

    float logXInput;
    float logYInput;

    float logPosX;
    float logPosY;

    float maxThrust;
    float minThrust;
    float landingThrust;
    float maxGAllowedEntry;

    std::array<float , 3> appliedVector;


    std::array<std::array<float,3> , 2> finVectors;

    std::array<float,3> targetLandingPosition;
    std::array<float,3> logEngineVector;
    std::array<float,3> logMoment;
    
    std::unique_ptr<StanleyController> Stanley;
    public:
    float gimbalX;
    float gimbalY;
    float dryMass , fuel , LOX , fuelConsumptionRate , LOXConsumptionRate ;

    std::array<float,3> engineState;
    float engineForce;
    //used to send data to simulation for fire animation, no physics is done
    float enginePower;

        Rocket(std::string config);

        Rocket(const Rocket& vehicle) = delete;

        Rocket& operator=(const Rocket& other);

        void init(string& configFile) override;

        void drag(float (*aeroArea)(float),float (*coefOfDrag)(float)) override;

        void lift(float (*aeroArea)(float),float (*coefOfLift)(float)) override;

        void applyEngineForce(std::array<float,2> twoDEngineRadians , float thrust);

        void updateState(float time,std::optional<controlPacks::variantPackets> controlInput) override;

        std::array<std::array<float , 3> , 2> getFinForceVectors();

        void applyFinForce(float xForce , float yForce);

        float getCurvature();

        bool fuelConsumption(float thrust);

        void engineGimbal(float gimbalTargetX , float gimbalTargetY);

        void initSensors() override;

        void updateFinPosition(std::pair<float,float> commands);

        float aeroArea(float angle);
        
        float coefOfDrag(float angle);

        float coefOfLift(float angle);

        inline std::array<float,3> getRadarPosition(){
            std::array<float,3> radar = {0,0,0};
            return radar;
        }
        
        ////kenetic + potential energy per unit mass
        //inline float getSpecificEnergy(){
        //    std::array<float,3> estimatedVelo =getEstimatedVelocity();
        //    float velo = vectorMag(estimatedVelo);
        //    return  (velo * velo)/2 + std::abs(gravitationalAcceleration * Zposition);
        //}
        //// kenetic + potential energy per unit mass in vector form {x,y,z}
        //inline std::array<float,3> getVectorizedEnergy(){
        //    std::array<float,3> estimatedVelo = getEstimatedVelocity();
        //    return {estimatedVelo[0]*estimatedVelo[0]/2 , estimatedVelo[1]*estimatedVelo[1]/2 , (estimatedVelo[2]*estimatedVelo[2] / 2) + std::abs(gravitationalAcceleration * getEstimatedPosition()[2])};
        //}


    //Vehicle &lookAhead( float lookAheadTime);

        
    std::vector<float> lookAhead(Rocket &rocket,float lookAheadTime , std::function<float(Rocket&)> valueToLog);

    void reentryBurn(loggedData *data = nullptr);

    void glideToTarget();

    void landingBurn();

    void setEntitiesPose(const poseState& pose) override;

    //only use prior to simulation
    //not a hard rule, wont be enforced
    inline void setPosition(float Xpos,float Ypos,float Zpos){
        Xposition = Xpos;
        Yposition = Ypos;
        Zposition = Zpos;
    }
    //only use prior to simulation
    //not a hard rule, wont be enforced
    inline void setVelocity(float Xvelo,float Yvelo,float Zvelo){
        Xvelocity = Xvelo;
        Yvelocity = Yvelo;
        Zvelocity = Zvelo;
    }

};

}

#endif