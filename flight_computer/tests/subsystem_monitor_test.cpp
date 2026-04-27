#include <gtest/gtest.h>
#include <subsystem_monitor.h>




class subsystemFixture : public ::testing::Test {
public:

    avionics::flight_health state;

    subsystem<&avionics::flight_health::hardware> subsys;
    
    subsystemFixture(): subsys(state){

    }
};


TEST_F(subsystemFixture, raiseFaultFlag){
    subsys.set_fault();
    subsys.set_boot(false);
    EXPECT_EQ(state.hardware.lifecycle.load(),static_cast<avionics::lc_type>(avionics::subsystem_lifecycle::lifecycle_fault));
    subsys.set_fault(false);
    EXPECT_EQ(state.hardware.lifecycle.load(),static_cast<avionics::lc_type>(avionics::subsystem_lifecycle::lifecycle_none));
}


TEST_F(subsystemFixture, raiseRunningFlag){
    subsys.set_running();
    subsys.set_boot(false);
    EXPECT_EQ(state.hardware.lifecycle.load(), static_cast<avionics::lc_type>(avionics::subsystem_lifecycle::lifecycle_running));
    subsys.set_running(false);
    EXPECT_EQ(state.hardware.lifecycle.load(), static_cast<avionics::lc_type>(avionics::subsystem_lifecycle::lifecycle_none));
}


TEST_F(subsystemFixture, multipleFlags){
    subsys.set_running();
    subsys.set_connected();
    subsys.set_connecting();
    subsys.set_fault();
    using namespace avionics;
    EXPECT_EQ(state.hardware.lifecycle.load(), (
        static_cast<lc_type>(subsystem_lifecycle::lifecycle_boot) |
        static_cast<lc_type>(subsystem_lifecycle::lifecycle_running) |
        static_cast<lc_type>(subsystem_lifecycle::lifecycle_connected)|
        static_cast<lc_type>(subsystem_lifecycle::lifecycle_connecting) |
        static_cast<lc_type>(subsystem_lifecycle::lifecycle_fault))
    );
}


TEST_F(subsystemFixture, multipleCallsToRaiseFlag){
    subsys.set_boot(false);
    for(int i = 0 ; i < 3 ; i++ ){
        subsys.set_connected();
        EXPECT_EQ(state.hardware.lifecycle.load(),static_cast<avionics::lc_type>(avionics::subsystem_lifecycle::lifecycle_connected));
    }
}


TEST_F(subsystemFixture, multipleCallsToLowerFlag){
    for(int i = 0 ; i < 3 ; i++ ){
        subsys.set_boot(false);
        EXPECT_EQ(state.hardware.lifecycle.load(),static_cast<avionics::lc_type>(avionics::subsystem_lifecycle::lifecycle_none));
    }
}


TEST_F(subsystemFixture, callsToLogError){
    for(int i = 0; i < 100 ; i++){
        subsys.increment_error();
        EXPECT_EQ(i+1,state.hardware.error_count);
    }
}


TEST_F(subsystemFixture, logRecentCallTime){
    for(uint64_t i = 0; i < 1000 ; i++){
        u_int64_t t = i * static_cast<uint64_t>(5000);
        subsys.set_last_call_us(t);
        EXPECT_EQ(state.hardware.last_update_us.load(),t);
    }
}


TEST_F(subsystemFixture, logRecentCallTimeCannotReverse){
    u_int64_t t = 5000;
    subsys.set_last_call_us(t);
    EXPECT_EQ(state.hardware.last_update_us.load(),t);
    t = 3000;
    subsys.set_last_call_us(t);
    EXPECT_NE(state.hardware.last_update_us.load(),t);
}