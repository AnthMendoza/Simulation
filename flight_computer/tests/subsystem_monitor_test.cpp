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
    EXPECT_EQ(state.hardware.lifecycle.load(),avionics::lifecycle_fault);
    subsys.set_fault(false);
    EXPECT_EQ(state.hardware.lifecycle.load(),avionics::lifecycle_none);
}

TEST_F(subsystemFixture, raiseRunningFlag){
    subsys.set_running();
    subsys.set_boot(false);
    EXPECT_EQ(state.hardware.lifecycle.load(),avionics::lifecycle_running);
    subsys.set_running(false);
    EXPECT_EQ(state.hardware.lifecycle.load(), avionics::lifecycle_none);
}

TEST_F(subsystemFixture, multipleFlags){
    subsys.set_running();
    subsys.set_connected();
    subsys.set_connecting();
    subsys.set_fault();
    using namespace avionics;
    EXPECT_EQ(state.hardware.lifecycle.load(), (lifecycle_boot | lifecycle_running | lifecycle_connected | lifecycle_connecting | lifecycle_fault));
}

TEST_F(subsystemFixture, multipleCallsToRaiseFlag){
    subsys.set_boot(false);
    for(int i = 0 ; i < 10 ; i++ ){
        subsys.set_connected();
        EXPECT_EQ(state.hardware.lifecycle.load(),avionics::lifecycle_connected);
    }
}

TEST_F(subsystemFixture, multipleCallsToLowerFlag){
    for(int i = 0 ; i < 10 ; i++ ){
        subsys.set_boot(false);
        EXPECT_EQ(state.hardware.lifecycle.load(),avionics::lifecycle_none);
    }
}