#pragma once
#include "../dynamics/vehicle.h"
#include "../control/droneControllerBase.h"
#include "../control/stateEstimationBase.h"
#include "simulation.h"
#include "../utility/utility.h"


namespace SimCore{

class droneSimulation : public simulation{
private:
    std::unique_ptr<droneControllerBase> controller;
    std::unique_ptr<stateEstimationBase> estimator;
    std::unique_ptr<Vehicle> vehicle;


    void step(float time) override{
        if (!vehicle || !estimator || !controller) {
            return;
        }
        time += config.timeStep;
        vehicle->addMoment({0,0,0.1});

        auto sensorPacket = vehicle->sensors->getSensorData();
        estimator->updateEstimation(time,sensorPacket);
        auto controlPacket = controller->update(time,estimator->getStateInfo());
        vehicle->updateState(time,controlPacket);
    }


    void startUp() override{

    }

    void logs() override{
        std::string vehicleString = vehicle->display();
        printDynamicDisplay(vehicleString);
    }

    

public:
template<typename TVehicle, typename TController, typename TEstimator>
//(vehicle , controller , estimator)
droneSimulation(std::unique_ptr<TVehicle> newVehicle,
                std::unique_ptr<TController> newController,
                std::unique_ptr<TEstimator> newEstimator) {
    static_assert(std::is_base_of_v<Vehicle, TVehicle>, 
                  "TVehicle must derive from Vehicle");
    static_assert(std::is_base_of_v<droneControllerBase, TController>, 
                  "TController must derive from droneControllerBase");
    static_assert(std::is_base_of_v<stateEstimationBase, TEstimator>, 
                  "TEstimator must derive from stateEstimationBase");
    
    vehicle = std::move(newVehicle);
    controller = std::move(newController);
    estimator = std::move(newEstimator);
}

    //setter
    template<typename T>
    void setDrone(std::unique_ptr<T> newVehicle){
        static_assert(std::is_base_of_v<Vehicle, T>, "T must derive from vehicle");
        vehicle = std::move(newVehicle);
    }

    template<typename T>
    void setController(std::unique_ptr<T> newController){
        static_assert(std::is_base_of_v<droneControllerBase, T>, "T must derive from droneControllerBase");
        controller = std::move(newController);
    }

    template<typename T>
    void setEstimator(std::unique_ptr<T> newEstimator){
        static_assert(std::is_base_of_v<stateEstimationBase, T>, "T must derive from stateEstimationBase");
        estimator = std::move(newEstimator);
    }

    //getter
    Vehicle* getDrone() const{
        return vehicle.get();
    }
    droneControllerBase* getController() const{ 
        return controller.get();
    }
    stateEstimationBase* getEstimator() const{
        return estimator.get();
    }

};

}
