#pragma once
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <iostream>
#include "../dynamics/vehicle.h"
#include "../control/droneControllerBase.h"
#include "../control/stateEstimationBase.h"
#include "../utility/time_manager.h"

namespace SimCore{

class simulation{
public:
    struct configStruct {
        bool realTime = true;            //true = simulation plays in real time, false runs simulation without clock timeout
        float timeStep = 0.001f;         //Fixed time step (seconds)
        float maxSimTime = 10.0f;        //Max simulation time, 0 = unlimited
        bool logging = true;             //Enable/disable data logging
        float logPostInterval = 1.0f;
    };
private:

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

        if(config.realTime){
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

protected:

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
        if(simThread.joinable()){
            simThread.join();
        }
    }



};

}
