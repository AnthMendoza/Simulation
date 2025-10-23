#include "../include/utility/time_manager.h"
#include <gtest/gtest.h>

using namespace SimCore;

TEST(timeManagerTest, numberOfTriggers){
    timeManager manager;
    float interval = 0.01f;
    manager.setInterval(interval);
    int duration = 10.0f;
    float expectedValue = duration/interval + 1;
    int count = 0;
    
    int numSteps = static_cast<int>(duration / interval);
    for(int i = 0; i <= numSteps; i++){
        float time = i * interval;
        if(manager.shouldTrigger(time)){
            count++;
        }
    }
    EXPECT_EQ(static_cast<int>(expectedValue), count);
}

TEST(timeManagerTest, firstTriggerAtInterval) {
    timeManager manager;
    float interval = 0.5f;
    manager.setInterval(interval);
    
    EXPECT_TRUE(manager.shouldTrigger(0.0f));
    EXPECT_FALSE(manager.shouldTrigger(0.3f));
    
    EXPECT_TRUE(manager.shouldTrigger(0.5f));
}

TEST(timeManagerTest, noTriggerBeforeInterval) {
    timeManager manager;
    float interval = 1.0f;
    manager.setInterval(interval);
    
    EXPECT_TRUE(manager.shouldTrigger(0.0f));
    EXPECT_FALSE(manager.shouldTrigger(0.5f));
    EXPECT_FALSE(manager.shouldTrigger(0.99f));
}

TEST(timeManagerTest, triggerAtExactMultiples) {
    timeManager manager;
    float interval = 0.25f;
    manager.setInterval(interval);
    
    EXPECT_TRUE(manager.shouldTrigger(0.25f));
    EXPECT_TRUE(manager.shouldTrigger(0.5f));
    EXPECT_TRUE(manager.shouldTrigger(0.75f));
    EXPECT_TRUE(manager.shouldTrigger(1.0f));
}

TEST(timeManagerTest, noDoubleTriggerSameTime) {
    timeManager manager;
    float interval = 0.5f;
    manager.setInterval(interval);
    
    EXPECT_TRUE(manager.shouldTrigger(0.5f));
 
    EXPECT_FALSE(manager.shouldTrigger(0.5f));
}



TEST(timeManagerTest, intervalChange) {
    timeManager manager;
    manager.setInterval(1.0f);
    
    EXPECT_TRUE(manager.shouldTrigger(1.0f));
    
    manager.setInterval(0.5f);
    
    EXPECT_TRUE(manager.shouldTrigger(1.5f));
}

TEST(timeManagerTest, nonUniformTimeSteps) {
    timeManager manager;
    float interval = 0.5f;
    manager.setInterval(interval);
    int count = 0;
    
    if(manager.shouldTrigger(0.0f)) count++;
    if(manager.shouldTrigger(0.3f)) count++;
    if(manager.shouldTrigger(0.7f)) count++; 
    if(manager.shouldTrigger(1.1f)) count++;  
    if(manager.shouldTrigger(1.3f)) count++;
    
    EXPECT_EQ(3, count);
}



