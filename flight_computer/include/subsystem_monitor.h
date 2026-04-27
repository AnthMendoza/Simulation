#ifndef SUBSYSTEM_MONITOR_H
#define SUBSYSTEM_MONITOR_H

#include <computer_state.h>
#include <atomic>
#include <buffer_types.h>

//***Not a type template***
//pointer to a member of flight_computer_health
//that is of type subsystem_health
template<avionics::subsystem_health avionics::flight_health::* member>

class subsystem{
    private:

    avionics::flight_computer_health& comp_health;

    void set_lifecycle_flag(avionics::subsystem_lifecycle state){
        auto& subsystem_member = comp_health.*member;
        auto& atomic_lc = subsystem_member.lifecycle;

        atomic_lc.fetch_or(state, std::memory_order_relaxed);
    }

    void clear_lifecycle_flag(avionics::subsystem_lifecycle state){
        auto& subsystem_member = comp_health.*member;
        auto& atomic_lc = subsystem_member.lifecycle;
        //using ~ to flip all bits then and to combine is a common way to flip a single bit.
        atomic_lc.fetch_and(~state, std::memory_order_relaxed); 
    }

    protected:
    //raises fault flag and changes health status to warning
    void set_fault(bool raise_flag = true){
        raise_flag ? set_lifecycle_flag(avionics::lifecycle_fault)
                   : clear_lifecycle_flag(avionics::lifecycle_fault);
    }

    void set_running(bool raise_flag = true){
    raise_flag ? set_lifecycle_flag(avionics::lifecycle_running)
                : clear_lifecycle_flag(avionics::lifecycle_running);
    }

    void set_initialized(bool raise_flag = true){
        raise_flag ? set_lifecycle_flag(avionics::lifecycle_initialized)
                    : clear_lifecycle_flag(avionics::lifecycle_initialized);
    }

    void set_boot(bool raise_flag = true){
        raise_flag ? set_lifecycle_flag(avionics::lifecycle_boot)
                    : clear_lifecycle_flag(avionics::lifecycle_boot);
    }

    void set_offline(bool raise_flag = true){
        raise_flag ? set_lifecycle_flag(avionics::lifecycle_offline)
                    : clear_lifecycle_flag(avionics::lifecycle_offline);
    }

    void set_connecting(bool raise_flag = true){
        raise_flag ? set_lifecycle_flag(avionics::lifecycle_connecting)
                    : clear_lifecycle_flag(avionics::lifecycle_connecting);
    }

    void set_connected(bool raise_flag = true){
        raise_flag ? set_lifecycle_flag(avionics::lifecycle_connected)
                    : clear_lifecycle_flag(avionics::lifecycle_connected);
    }


    public:

    subsystem(avionics::flight_health& _comp_health): comp_health(_comp_health){

    }
};

#endif