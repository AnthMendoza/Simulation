
#include <array>
#include <cmath>
#include <memory>
#include <cmath>
#include "../../include/dynamics/rocket.h"
#include "../../include/core/forceApplied.h"
#include "../../include/dynamics/vehicle.h"
#include "../../include/core/vectorMath.h"
#include "../../include/dynamics/aero.h"
#include "../../include/core/RungeKutta.h"
#include "../../include/core/odeIterator.h"
#include "../../include/core/rotationMatrix.h"
#include "../../include/core/getRotation.h"
#include "../../include/control/control.h"
#include "../../include/subsystems/sensors.h"
#include "../../include/sim/toml.h"

namespace SimCore{


Rocket& Rocket::operator=(const Rocket& other){
    if (this == &other) return *this; // handles self assignment || example a=a
    Vehicle::operator=(other);

    if (other.Stanley)
        Stanley = std::make_unique<StanleyController>(*other.Stanley);

    dryMass = other.dryMass;
    fuel = other.fuel;
    LOX = other.LOX;
    mass = other.mass;
    fuelConsumptionRate = other.fuelConsumptionRate;
    LOXConsumptionRate = other.LOXConsumptionRate;
    MOI = other.MOI;
    targetLandingPosition = other.targetLandingPosition;
    gimbalDamping = other.gimbalDamping;
    gimbalPGain = other.gimbalPGain;
    gimbalIGain = other.gimbalIGain;
    gimbalDGain = other.gimbalDGain;
    finDamping = other.finDamping;
    finPGain = other.finPGain;
    finIGain = other.finIGain;
    finDGain = other.finDGain;
    maxFinAcceleration = other.maxFinAcceleration;
    maxGimbalAcceleration = other.maxGimbalAcceleration;
    maxGAllowedEntry = other.maxGAllowedEntry;
    centerOfPressure = other.centerOfPressure;
    cogToEngine = other.cogToEngine;
    maxThrust = other.maxThrust;
    minThrust = other.minThrust;
    landingThrust = other.landingThrust;
    maxGimbalAngle = other.maxGimbalAngle;

    gimbalErrorX = other.gimbalErrorX;
    gimbalErrorY = other.gimbalErrorY;
    gimbalX = other.gimbalX;
    gimbalY = other.gimbalY;
    gimbalVelocityX = other.gimbalVelocityX;
    gimbalVelocityY = other.gimbalVelocityY;
    twoDAngle = other.twoDAngle;
    error = other.error;
    sumOfGimbalErrorX = other.sumOfGimbalErrorX;
    sumOfGimbalErrorY = other.sumOfGimbalErrorY;
    vehicleYError = other.vehicleYError;
    sumOfVehicleYError = other.sumOfVehicleYError;
    vehicleXError = other.vehicleXError;
    sumOfVehicleXError = other.sumOfVehicleXError;
    logXInput = other.logXInput;
    logYInput = other.logYInput;
    logPosX = other.logPosX;
    logPosY = other.logPosY;
    logMoment = other.logMoment;
    finErrorX = other.finErrorX;
    finErrorY = other.finErrorY;
    finX = other.finX;
    finY = other.finY;
    sumOfFinErrorX = other.sumOfFinErrorX;
    sumOfFinErrorY = other.sumOfFinErrorY;
    finVelocityX = other.finVelocityX;
    finVelocityY = other.finVelocityY;
    return *this;
}


Rocket::Rocket(std::string config){
    configFile = config;
}




void Rocket::init(string& configFile){
    toml::tomlParse vehicleParse;
    vehicleParse.parseConfig( configFile,"vehicle");

    initSensors();

    // Controller
    Stanley = std::make_unique<StanleyController>(
        vehicleParse.getFloat("StanleyGain"),
        vehicleParse.getFloat("maxSteeringAngle")
    );

    // Mass parameters
    dryMass = vehicleParse.getFloat("dryMass");
    fuel = vehicleParse.getFloat("initFuel");
    LOX = vehicleParse.getFloat("initLOX");
    mass = dryMass + fuel + LOX;

    fuelConsumptionRate =   vehicleParse.getFloat("consumptionRateFuel");
    LOXConsumptionRate =    vehicleParse.getFloat("consumptionRateLOX");


    // Landing target
    auto landingPos = vehicleParse.getArray("targetLandingPosition");
    targetLandingPosition[0] = landingPos[0];
    targetLandingPosition[1] = landingPos[1];
    targetLandingPosition[2] = landingPos[2];
    // Gimbal PID
    gimbalDamping = vehicleParse.getFloat("gimbalDamping");
    gimbalPGain =   vehicleParse.getFloat("gimbalPGain");
    gimbalIGain =   vehicleParse.getFloat("gimbalIGain");
    gimbalDGain =   vehicleParse.getFloat("gimbalDGain");

    // Fin PID
    finDamping =            vehicleParse.getFloat("finDamping");
    finPGain =              vehicleParse.getFloat("finPGain");
    finIGain =              vehicleParse.getFloat("finIGain");
    finDGain =              vehicleParse.getFloat("finDGain");
    maxFinAcceleration =    vehicleParse.getFloat("maxFinAcceleration");
    maxGimbalAcceleration = vehicleParse.getFloat("maxGimbalAcceleration");

    maxGAllowedEntry =  vehicleParse.getFloat("maxGAllowedEntry");
    centerOfPressure =  vehicleParse.getFloat("centerOfPressure");
    cogToEngine =       vehicleParse.getFloat("cogToEngine");// meters use this to calculate the moment created by the engine this is negative becuase center of pressure on the opposite side of the COG is positive
    maxThrust =         vehicleParse.getFloat("maxThrust");
    minThrust = maxThrust *.5; //newtons, 65 percent of max;
    landingThrust = maxThrust * .95 ;
    maxGimbalAngle = 30  * 3.1415926535f / 180.0f;// degrees
   


    
    gimbalErrorX = 0;
    gimbalErrorY = 0;
    gimbalX = 0;
    gimbalY = 0;
    gimbalVelocityX = 0;
    gimbalVelocityY = 0;

    twoDAngle = {0,0};
    error = 0;

    sumOfGimbalErrorX = 0;
    sumOfGimbalErrorY = 0;

    vehicleYError = 0;
    sumOfVehicleYError = 0;

    vehicleXError = 0;
    sumOfVehicleXError = 0;

    logXInput = 0;
    logYInput = 0;
    logPosX = 0;
    logPosY = 0;
    

    logMoment = {0,0,0};

    finErrorX = 0;
    finErrorY = 0;
    finX = 0;
    finY = 0;
    sumOfFinErrorX = 0;
    sumOfFinErrorY = 0;
    
    finVelocityX = 0;
    finVelocityY = 0;
    Vehicle::init(configFile);

}


void Rocket::drag(float (*aeroArea)(float),float (*coefOfDrag)(float)){
    Vehicle::drag(aeroArea,coefOfDrag);

}




void Rocket::initSensors(){
    
    Vehicle::initSensors();

}


void Rocket::lift(float (*aeroArea)(float),float (*coefOfDrag)(float)){
    Vehicle::lift(aeroArea,coefOfDrag);
}   

void Rocket::applyEngineForce(std::array<float,2> twoDEngineRadians , float thrust){
    
    if(fuelConsumption(thrust) == false) return;

    thrust = -thrust;



    Matrix3x3 rotX = rotationMatrixX(twoDEngineRadians[0]);
    Matrix3x3 rotY = rotationMatrixY(twoDEngineRadians[1]);

    std::array<float,3> engineVector = {0,0,1};

    Eigen::Vector3d v1(engineVector[0], engineVector[1], engineVector[2]);

    Matrix3x3 combined = rotX * rotY;

    engineVector = combined.rotate(engineVector);


    std::array<float , 3> vehicleStateAntiparallel = vehicleState;
    // reverses the vector so that we can apply a thrust in the oposite direction

    for(int i = 0; i<3 ; i++){
        vehicleStateAntiparallel[i] = vehicleStateAntiparallel[i]; 
    } 

    Eigen::Vector3d v2( vehicleStateAntiparallel[0],
                        vehicleStateAntiparallel[1],
                        vehicleStateAntiparallel[2]);
    
    Eigen::Vector3d rotation = getRotationAngles(v1, v2);


    std::array<float,3> vehicleStateAngles = {static_cast<float>(rotation(0)) , static_cast<float>(rotation(1)) , static_cast<float>(rotation(2))};

    Matrix3x3 rotXVehicle = rotationMatrixX(vehicleStateAngles[0]); 
    Matrix3x3 rotYVehicle = rotationMatrixY(vehicleStateAngles[1]); 
    Matrix3x3 rotZVehicle = rotationMatrixZ(vehicleStateAngles[2]); 


    Matrix3x3 realign = rotXVehicle * rotYVehicle * rotZVehicle;

    engineVector = realign.rotate(engineVector);

    engineState = engineVector;


    enginePower = abs(thrust) /  maxThrust;

    setInBounds(enginePower , 0.0f , 1.0f);
    

    engineVector[0] = engineVector[0] *-thrust;
    engineVector[1] = engineVector[1] *-thrust;
    engineVector[2] = engineVector[2] *-thrust;

    
    
    addForce(engineVector);
    
    



    if(twoDEngineRadians[0] != 0 && twoDEngineRadians[1]  != 0){
        //addMoment(forceToMoment(engineVector, vehicleState , cogToEngine));
        //logMoment = forceToMoment(engineVector, vehicleState , cogToEngine);
    }
    
    appliedVector[0] = vehicleState[0] * cogToEngine;
    appliedVector[1] = vehicleState[1] * cogToEngine;
    appliedVector[2] = vehicleState[2] * cogToEngine;

    engineForce = abs(thrust);



}


void Rocket::updateState(){    

    Vehicle::updateState();

    engineState = {0,0,0};

    enginePower = 0;

    appliedVector = {0,0,0};

    //reset finVectors::: might remove 
    finVectors[0] =  {0,0,0};
    finVectors[1] =  {0,0,0};



}





std::array<std::array<float , 3> , 2> Rocket::getFinForceVectors(){


    std::array<float,3> Xfin = {1,0,0};
    std::array<float,3> Yfin = {0,1,0};
    std::array<float,3> zrefrance = {0,0,1};

    Eigen::Vector3d v1( zrefrance[0],
                        zrefrance[1],
                        zrefrance[2]);


    Eigen::Vector3d v2( vehicleState[0],
                        vehicleState[1],
                        vehicleState[2]);
    
    Eigen::Vector3d rotation = getRotationAngles(v1, v2);

    //get vehicle rotation relative to a refrance vector then alter our preset force application vectors


    std::array<float,3> vehicleStateAngles = {static_cast<float>(rotation(0)) , static_cast<float>(rotation(1)) , static_cast<float>(rotation(2))};


    Matrix3x3 rotX = rotationMatrixX(vehicleStateAngles[0]);
    Matrix3x3 rotY = rotationMatrixY(vehicleStateAngles[1]);
    Matrix3x3 rotZ = rotationMatrixZ(vehicleStateAngles[2]);


    Matrix3x3 realign = rotX * rotY * rotZ;


    Xfin = realign.rotate(Xfin);
    Yfin = realign.rotate(Yfin);


    std::array<std::array<float , 3> , 2> finVectors = {Xfin , Yfin};

    return finVectors;

}

void Rocket::applyFinForce(float xForce , float yForce){
    auto forceVectors = getFinForceVectors();
    forceVectors[0] = normalizeVector(forceVectors[0]);
    for(int i = 0 ; i < 3 ; i++) {
         forceVectors[0][i] = forceVectors[0][i] * xForce;
    }
    if(forceVectors[0][0] != 0 && forceVectors[0][1] != 0 && forceVectors[0][2] != 0){
        addForce(forceVectors[0]);
        addMoment(forceToMoment(forceVectors[0], vehicleState , centerOfPressure));
    }          


    if(forceVectors[1][0] != 0 && forceVectors[1][1] != 0 && forceVectors[1][2] != 0){
        addForce(forceVectors[1]);
        addMoment(forceToMoment(forceVectors[1], vehicleState , centerOfPressure));
    }          

}

void Rocket::updateFinPosition(std::pair<float,float> commands){
    setInBounds( commands.first , - maxGimbalAngle ,  maxGimbalAngle);
    setInBounds( commands.second , - maxGimbalAngle ,  maxGimbalAngle);
    
    finErrorX = commands.first -  finX;
    finErrorY = commands.second - finY;

    float XInput = PID(commands.first, finX , finErrorX , sumOfFinErrorX,  timeStep , finPGain , finIGain , finDGain);

    float YInput = PID(commands.second, finY , finErrorY , sumOfFinErrorY ,  timeStep , finPGain , finIGain , finDGain);


    setInBounds( XInput , -1.0f , 1.0f);
    setInBounds( YInput , -1.0f , 1.0f);


    logXInput = ((XInput * maxFinAcceleration) - finDamping * finVelocityX);
    logYInput = ((XInput * maxFinAcceleration) - finDamping * finVelocityX);
    
    if (finVelocityX > 1)finVelocityX += ((XInput * maxFinAcceleration) - finDamping * finVelocityX * finVelocityX) *  timeStep;
    else finVelocityX += ((XInput * maxFinAcceleration) - finDamping * finVelocityX) *  timeStep;

    if (finVelocityY > 1)finVelocityY += ((YInput * maxFinAcceleration) - finDamping * finVelocityY * finVelocityY) *  timeStep;
    else finVelocityY += ((YInput * maxFinAcceleration) - finDamping * finVelocityY) *  timeStep;

    finX += finVelocityX *  timeStep;
    finY += finVelocityY *  timeStep;
}



//No longer in use
float Rocket::getCurvature(){
    
    std::array<float ,3> velocity = {Xvelocity , Yvelocity , Zvelocity};
    std::array<float,3> veloCrossAccel = {0,0,0};

    //vectorCrossProduct(velocity , acceleration , veloCrossAccel);
    
    //return vectorMag(veloCrossAccel) / pow( vectorMag(velocity) , 3 );
    return 0;

    
}

bool Rocket::fuelConsumption(float thrust){ 
    //returns true if vehicle has enough fuel to apply requested force. false should stop force application
    // numOfEngineOn can be fractional ex .8 is 80 percent thrust of 1 engine, or 3 is 3 engine at full power

    float numOfEngineOn = thrust /  maxThrust;

    float consumptionDuringTimeFuel  = fuelConsumptionRate * numOfEngineOn *  timeStep;
    float consumptionDuringTimeLOX  = LOXConsumptionRate * numOfEngineOn *  timeStep;

    if(fuel - consumptionDuringTimeFuel < 0 || LOX - consumptionDuringTimeLOX < 0) return false;

    fuel += -consumptionDuringTimeFuel;
    LOX += -consumptionDuringTimeLOX;

    mass = dryMass + fuel + LOX;

    return true;

}



void Rocket::engineGimbal(float gimbalTargetX , float gimbalTargetY){

    setInBounds( gimbalTargetX , - maxGimbalAngle ,  maxGimbalAngle);
    setInBounds( gimbalTargetY , - maxGimbalAngle ,  maxGimbalAngle);
    
    gimbalErrorX = gimbalTargetX - gimbalX;
    gimbalErrorY = gimbalTargetY - gimbalY;

    float XInput = PID(gimbalTargetX , gimbalX , gimbalErrorX , sumOfGimbalErrorX ,  timeStep , gimbalPGain , gimbalIGain , gimbalDGain);

    float YInput = PID(gimbalTargetY , gimbalY , gimbalErrorY , sumOfGimbalErrorY ,  timeStep , gimbalPGain , gimbalIGain , gimbalDGain);

    
    setInBounds( XInput , -1.0f , 1.0f);
    setInBounds( YInput , -1.0f , 1.0f);


    logXInput = ((XInput * maxGimbalAcceleration) - gimbalDamping * gimbalVelocityX);
    logYInput = ((XInput * maxGimbalAcceleration) - gimbalDamping * gimbalVelocityX);
    
    if(gimbalVelocityX > 1) gimbalVelocityX += ((XInput * maxGimbalAcceleration) - gimbalDamping * gimbalVelocityX * gimbalVelocityX) *  timeStep;
    else gimbalVelocityX += ((XInput * maxGimbalAcceleration) - gimbalDamping * gimbalVelocityX) *  timeStep;

    if(gimbalVelocityY > 1) gimbalVelocityY += ((YInput * maxGimbalAcceleration) - gimbalDamping * gimbalVelocityY * gimbalVelocityY) *  timeStep; 
    else  gimbalVelocityY += ((YInput * maxGimbalAcceleration) - gimbalDamping * gimbalVelocityY) *  timeStep; 

    gimbalX += gimbalVelocityX *  timeStep;
    gimbalY += gimbalVelocityY *  timeStep;
    
}


//Glide initation can be triggered based on the slope of the spscific energy. negavtive slope means drag is absorbing energy

void Rocket::glideToTarget(){

    //if(rocket.reentr  == false) return;
    auto positionEsitmated = getEstimatedPosition();
    std::array<float,2> xyDelta = { targetLandingPosition[0] - positionEsitmated[0],
                                    targetLandingPosition[1] - positionEsitmated[1]};

    std::array<float,3> targetVector = {0,0,-1};
    
    std::array<float,2> headingError = {vectorAngleBetween(vehicleState , targetVector) ,vectorAngleBetween(vehicleState , targetVector)};

    auto command = Stanley->computeSteering(headingError[0], xyDelta[0], getVelocity());

    float xforce = getVelocity() * command;

    applyFinForce(xforce , 0);
    
}





void Rocket::landingBurn(){

    if(reentry == false) return;
    if(0 != iterations % 20 && landingInProgress){
        engineGimbal( PIDOutputY , PIDOutputX );
        applyEngineForce(landingGimbalDirection, landingRequiredThrust);
        return;
    }
    std::array<float,3> velocity = getEstimatedVelocity();
    std::array<float,3> rotation = getEstimatedRotation();
    
    float landingAccelZ = ((velocity[2] * velocity[2]) / (2 * Zposition)) - gravitationalAcceleration;
    float landingBurnDuration =  velocity[2] / landingAccelZ;

    float landingAccelX = velocity[0]/ landingBurnDuration;

    float landingAccelY = velocity[1]/ landingBurnDuration;
    // as landing duration approches 0 landing acceleration X and Y grows rapidly 
    // lim 1/x as x approches +0 is infinity 
    if(landingBurnDuration < 1.0f ){ 
        landingAccelX = 0;
        landingAccelY = 0;
    }
    
    float landingForceX = landingAccelX * mass;

    float landingForceY = landingAccelY * mass;

    float landingForceZ = landingAccelZ * mass;

    landingRequiredThrust = sqrtf(landingForceX * landingForceX + landingForceY * landingForceY + landingForceZ * landingForceZ);

    if(landingRequiredThrust>=  landingThrust){
        landingInProgress = true;
    }

    if(landingRequiredThrust <=  minThrust || landingInProgress == false){
        landingInProgress = false; 
        return;
    }

    if(landingRequiredThrust >  maxThrust) landingRequiredThrust =  maxThrust;
    

    std::array<float,3> forceVector = {landingForceX , landingForceY , landingForceZ};


    std::array<float , 3> directionVector = normalizeVector(forceVector);

    landingGimbalDirection= {0,0};
        
    std::array<float,2> twoDState = {rotation[1], rotation[2]};

    std::array<float,2> targetState = {directionVector[1] , directionVector[2]};


    float error = twodAngleDiffrence( twoDState, targetState); 

    error = error;

    twoDAngle = twoDState;

    PIDOutputY = PID(0,error,vehicleYError, sumOfVehicleYError,  timeStep , 2 ,0 , 1);
    
    //std::cout<< twoDState[0] << "," << twoDState[1] << "  " << targetState[0] << "," << targetState[1]<< "   "<< error  << "   " << PIDOutputY << "\n";

    twoDState = {rotation[0] , rotation[2]};

    targetState = {directionVector[0] , directionVector[2]};

    error  = twodAngleDiffrence( twoDState, targetState);

    PIDOutputX = PID(0,error,vehicleXError, sumOfVehicleXError,  timeStep , 2 ,0, 1);



    engineGimbal( PIDOutputY , PIDOutputX );


    landingGimbalDirection[1] = -gimbalY;
    landingGimbalDirection[0] = gimbalX;


    applyEngineForce(landingGimbalDirection, landingRequiredThrust);
}


std::vector<float> Rocket::lookAhead(Rocket &rocket,float lookAheadTime , std::function<float(Rocket&)> valueToLog){
    //make a copy of the rocket so that we can keep current conditions on the main vehicle and test parameters on the look ahead
    int initalIterations = rocket.iterations;
    float limit = lookAheadTime/ timeStep;
    std::vector<float> log;
    while(rocket.iterations - initalIterations < limit && rocket.Zposition > 0){
        
        rocket.drag(aeroAreaRocket,coefOfDragRocket);
        rocket.lift(aeroAreaRocket,coefOfLiftRocket);
        rocket.updateState();
        log.push_back(valueToLog(rocket));
        rocket.iterations++;
    }

    return log;
}




// make sure sensors play well with lookahead
void Rocket::reentryBurn(loggedData *data){
    
    if(reentry == true) return;

    if(Zposition < 55000 && reentry == false){

        reentry = true;
        float currentMaxGForce =  maxGAllowedEntry + 1;
        float lastMaxGForce = currentMaxGForce;
        //main loop moves forward 1 second every lookAHead cylce
        float stepInterval = 1;
        std::array<float,2> direction = {0,0};
        float count = 0;
        while(currentMaxGForce >  maxGAllowedEntry){
            Rocket lookAheadRocket(configFile);
            lookAheadRocket = *this;
            lookAheadRocket.initSensors();
            std::vector<float> gForces = lookAhead(lookAheadRocket , 105 , [](Rocket &r) { return r.gForce; }); // 105 is the lookahead time in seconds. this may be stoppped earlier if the vehicle hits the ground
            if(gForces.size()!= 0)currentMaxGForce = *std::max_element(gForces.begin(),gForces.end());
            else currentMaxGForce =  maxGAllowedEntry + 1;
            if(currentMaxGForce <  maxGAllowedEntry && count > 0) return;
            if(currentMaxGForce > lastMaxGForce && count > 0) return;
            lastMaxGForce = currentMaxGForce;
            float currentIteration = iterations;
            count++;
            while(iterations < currentIteration + stepInterval/ timeStep && Zposition > 0){
                drag(aeroAreaRocket,coefOfDragRocket);
                lift(aeroAreaRocket,coefOfLiftRocket);
                applyEngineForce(direction ,  maxThrust * .6 * 3); 
                finVectors = getFinForceVectors();
                if(data != nullptr) data->logRocketPosition(*this);
                updateState();
                iterations++;
            }
        }
    }
}


void Rocket::setEntityPose(quaternionVehicle pose){

}


}