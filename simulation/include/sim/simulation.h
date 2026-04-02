#pragma once
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <iostream>
#include "../dynamics/vehicle.h"
#include "../control/droneControllerBase.h"
#include "../control/stateEstimationBase.h"
#include <util/

namespace SimCore{

class simulation{
public:

    enum class SimMode {
    RealTime,
    TimeIndependent,  
    ExternalStep //external Step enables simulationByStep which runs until deltaT.
    };

    struct configStruct {
        SimMode mode = SimMode::RealTime;
        float timeStep = 0.001f;         //Fixed time step (seconds)
        float maxSimTime = 10.0f;        //Max simulation time, 0 = unlimited
        bool logging = true;             //Enable/disable data logging
        float logPostInterval = 0.25f;
    };
protected:

    std::thread simThread;
    std::atomic<bool> running{false};
    std::atomic<bool> stopRequested{false};
    std::unique_ptr<timeManager> manager;
    mutable std::mutex dataMutex;

    virtual void step(float time) = 0 ;

    virtual void startUp() = 0;

    virtual void logs() = 0;

    virtual void runLoopRealTime(){
        running = true;
        float time = 0.0f;
        while (!stopRequested) {
            auto start = std::chrono::steady_clock::now();
            step(time);

            auto end = std::chrono::steady_clock::now();
            auto elapsed = end - start;

            auto timeStepChrono = std::chrono::duration<float>(config.timeStep);
            auto sleepTime = timeStepChrono - elapsed;

            if (sleepTime.count() > 0) {
                std::this_thread::sleep_for(sleepTime);
            }
            
            if(config.logging && manager->shouldTrigger(time)){
                logs();
            }

            timeConstraint(time);
            time += config.timeStep;
        }
        running = false;
        stopRequested = false;
    }

    virtual void runLoop(){
        running = true;
        float time = 0.0f;
        while (!stopRequested){
            step(time);
            time += config.timeStep;
            timeConstraint(time);
        }
        running = false;
        stopRequested = false;
    }

    virtual void loopDirector(){

        startUp();

        if(config.mode == SimMode::ExternalStep){
            simulationByStep();
            return;
        }

        if(config.mode == SimMode::RealTime){
            runLoopRealTime();
            return;
        }
        runLoop();
    }

    void timeConstraint(float time){
        if( config.maxSimTime == 0.0f ){
            return;
        }
        if(time >= config.maxSimTime){
            stop();
        }
    }


    configStruct config;

public: 
    simulation(){
        manager = std::make_unique<timeManager>(config.logPostInterval);
    }

    ~simulation() = default;

    void configure(std::function<void(configStruct&)> modify) {
        std::lock_guard<std::mutex> lock(dataMutex);
        modify(config);
        manager->setInterval(config.logPostInterval);
    }


    virtual void run() {
        if(running){
            std::cerr << "\n Simulation is already running \n";
            return;
        }
        
        stopRequested = false;
        if(simThread.joinable()){
            simThread.join();
        }
        simThread = std::thread(&simulation::loopDirector, this);
    }

    void stop(){
        stopRequested = true;
        if (std::this_thread::get_id() == simThread.get_id()){
            return;
        }
        if(simThread.joinable()){
            simThread.join();
        }
    }

    void wait() {
        if (simThread.joinable()) {
            simThread.join();
        }
    }

    private:

    std::condition_variable flag;
    std::atomic<bool> startFlag {false};
    std::mutex stepMutex;
    float deltaTime;

    public:

    //calls simulation steps based on an externalClock
    //defined for unrealEngine use;
    void simulationByStep(){
        if(config.mode != SimMode::ExternalStep){
            std::cout<< "\n Simulation not configured for SimulationByStep()\n";
            return;
        }

        float simulationTime = 0;
        float deltaTime = 0;
        
        while(!stopRequested){
            float localDeltaTime;
            {
                std::unique_lock<std::mutex> lock(stepMutex);

                flag.wait(lock,[&]{ return startFlag || stopRequested; });

                startFlag = false;
                localDeltaTime = deltaTime;
            } 

            float stopTime = simulationTime + localDeltaTime;
            while(simulationTime < stopTime ){
                
                step(simulationTime);

                if(config.logging && manager->shouldTrigger(simulationTime)){
                    logs();
                }

                simulationTime += config.timeStep;
            }

        }

        running = false;
        stopRequested = false;

    }

    void manualStep(float _deltaTime){

        {
            std::lock_guard<std::mutex> lock(stepMutex);

            deltaTime = _deltaTime;
            startFlag = true;
        }

        flag.notify_one();
        
    }


};

}
