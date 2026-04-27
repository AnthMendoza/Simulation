#ifndef SUBSYSTEM_MONITOR_H
#define SUBSYSTEM_MONITOR_H

#include <computer_state.h>
#include <atomic>
#include <buffer_types.h>
#include <optional>

#ifdef TESTING
#include <gtest/gtest_prod.h>
#endif

//***Not a type template***
//pointer to a member of flight_health
//that is of type subsystem_health
template<avionics::subsystem_health avionics::flight_health::* member>

class subsystem{
    #ifdef TESTING
    FRIEND_TEST(subsystemFixture, raiseFaultFlag);
    FRIEND_TEST(subsystemFixture, raiseRunningFlag);
    FRIEND_TEST(subsystemFixture, multipleFlags);
    FRIEND_TEST(subsystemFixture, multipleCallsToRaiseFlag);
    FRIEND_TEST(subsystemFixture, multipleCallsToLowerFlag);
    FRIEND_TEST(subsystemFixture, callsToLogError);
    FRIEND_TEST(subsystemFixture, logRecentCallTime);
    FRIEND_TEST(subsystemFixture, logRecentCallTimeCannotReverse);
    #endif
    private:

    avionics::flight_health& comp_health;

    void set_lifecycle_flag(avionics::subsystem_lifecycle state){
        auto& subsystem_member = comp_health.*member;
        auto& atomic_lc = subsystem_member.lifecycle;

        atomic_lc.fetch_or(static_cast<avionics::lc_type>(state), std::memory_order_relaxed);
    }

    void clear_lifecycle_flag(avionics::subsystem_lifecycle state){
        auto& subsystem_member = comp_health.*member;
        auto& atomic_lc = subsystem_member.lifecycle;
        //using ~ to flip all bits then and to combine is a common way to flip a single bit.
        atomic_lc.fetch_and(~static_cast<avionics::lc_type>(state), std::memory_order_relaxed); 
    }

    protected:
    //raises fault flag and changes health status to warning
    void set_fault(bool raise_flag = true){
        raise_flag ? set_lifecycle_flag(avionics::subsystem_lifecycle::lifecycle_fault)
                   : clear_lifecycle_flag(avionics::subsystem_lifecycle::lifecycle_fault);
    }

    void set_running(bool raise_flag = true){
    raise_flag ? set_lifecycle_flag(avionics::subsystem_lifecycle::lifecycle_running)
                : clear_lifecycle_flag(avionics::subsystem_lifecycle::lifecycle_running);
    }

    void set_initialized(bool raise_flag = true){
        raise_flag ? set_lifecycle_flag(avionics::subsystem_lifecycle::lifecycle_initialized)
                    : clear_lifecycle_flag(avionics::subsystem_lifecycle::lifecycle_initialized);
    }

    void set_boot(bool raise_flag = true){
        raise_flag ? set_lifecycle_flag(avionics::subsystem_lifecycle::lifecycle_boot)
                    : clear_lifecycle_flag(avionics::subsystem_lifecycle::lifecycle_boot);
    }

    void set_offline(bool raise_flag = true){
        raise_flag ? set_lifecycle_flag(avionics::subsystem_lifecycle::lifecycle_offline)
                    : clear_lifecycle_flag(avionics::subsystem_lifecycle::lifecycle_offline);
    }

    void set_connecting(bool raise_flag = true){
        raise_flag ? set_lifecycle_flag(avionics::subsystem_lifecycle::lifecycle_connecting)
                    : clear_lifecycle_flag(avionics::subsystem_lifecycle::lifecycle_connecting);
    }

    void set_connected(bool raise_flag = true){
        raise_flag ? set_lifecycle_flag(avionics::subsystem_lifecycle::lifecycle_connected)
                    : clear_lifecycle_flag(avionics::subsystem_lifecycle::lifecycle_connected);
    }

    void increment_error(){
        auto& subsystem_member = comp_health.*member;
        subsystem_member.error_count.fetch_add(1);
    }
    
    void set_last_call_us(uint64_t time_us) {
        auto& atomic = (comp_health.*member).last_update_us;

        uint64_t prev = atomic.load(std::memory_order_relaxed);

        if(time_us > prev)[[likely]]{
            atomic.store(time_us, std::memory_order_relaxed);
        }
    }

    public:

    subsystem(avionics::flight_health& _comp_health): comp_health(_comp_health){

    }
};

#endif
