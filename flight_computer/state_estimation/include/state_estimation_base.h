#ifndef STATE_ESTIMATION_BASE_H
#define STATE_ESTIMATION_BASE_H
#include "../../include/thread_manager.h"
#include "../../include/scheduler_tasks.h"

namespace avionics::estimation{

class estimation_base: public thread_manager{

    private:
    scheduler estimator_scheduler;


    protected:
    
    void initialize_base(){
        std::vector<scheduler_tasks> tasks = get_routine();
        for (auto& task : tasks){
            estimator_scheduler.add_manager(task.interval_s,task.callback);
        }
        set_scheduler(estimator_scheduler);
    }

    void thread_proccess() override{
        estimator_scheduler(start_to_recent_call_time());
    }

    void thread_startup_proccess() override{
        initialize_base();
    }


    virtual std::vector<avionics::scheduler_tasks> get_routine() = 0;


    public:


};

}


#endif