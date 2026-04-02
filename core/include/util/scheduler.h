#pragma once
#include "time_manager.h"
#include <vector>
#include <queue>
#include <unordered_set>
#include <functional>


class scheduler{
private:
    //Set of indices that are currently in the queue
    std::unordered_set<int> queued_indices;
    
    //float = overdue time
    std::priority_queue<
        std::pair<float, int>,
        std::vector<std::pair<float, int>>,
        std::less<std::pair<float, int>>
    > task_queue;
    
    std::vector<std::pair<SimCore::timeManager, std::function<void(float)>>> managers;

public:

    void add_manager(float interval, std::function<void(float)> callback){
        SimCore::timeManager manager(interval);
        managers.emplace_back(manager, callback);
    }

    void add_manager(float interval, std::function<void()> callback){
        SimCore::timeManager manager(interval);
        managers.emplace_back(manager, [callback](float time){callback();});
    }
    

    void operator()(float time){
        for(int i = 0; i < managers.size(); i++){

            auto& manager = managers[i].first;

            float last_trigger = manager.getLastTriggerTime();

            float interval = manager.getInterval();
            

            if(last_trigger + interval <= time){
                float overSchedule = time - (last_trigger + interval);
                
                if(queued_indices.find(i) == queued_indices.end()){
                    task_queue.push({overSchedule, i});
                    queued_indices.insert(i);
                }
            }
        }
        

        while(!task_queue.empty()){

            auto [overSchedule, index] = task_queue.top();

            task_queue.pop();

            queued_indices.erase(index);
                
            auto& manager = managers[index].first;
            
            managers[index].second(time);
            
            managers[index].first.shouldTrigger(time);
        }
    }

    //if vector manager is empty return value = std::numeric_limits<float>::max() 
    float time_until_next_call(float time){
        float minumum_time = std::numeric_limits<float>::max();
        for(auto& manager : managers){

            auto& time_manager = manager.first;
            float next_trigger_time = time_manager.getLastTriggerTime() + time_manager.getInterval();
            float dt = next_trigger_time - time;
            minumum_time = std::min(minumum_time,dt);

        }
        return minumum_time;
    }



    auto get_next_time_callback(){
        auto lambda = [this](float time){
            return time_until_next_call(time);
        };
        return lambda;
    }
    

    void clear(){
        while(!task_queue.empty()){
            task_queue.pop();
        }
        queued_indices.clear();
    }
    

    size_t pending_tasks() const {
        return task_queue.size();
    }
};
