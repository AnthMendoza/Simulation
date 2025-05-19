#ifndef MYSIM_H
#define MYSIM_H
#include "vehicle.h"
#include "rocket.h"
#include <mutex>
namespace SimCore{
//#pragma pack(push,1)
struct unrealData{
    float timeStamp;
    float position[3];
    float velocity[3];
    float rotation[3]; //format undecided
};
//#pragma pack(pop)


class unreal{
    private:
    unrealData packet;
    Rocket unrealVehicle;
    std::mutex packetMutex;
    void setPacket();
    void getPacket(unrealData &dataPacket);
    void iterator(float deltaTime);
    public:
    unreal(const char* ConfigPath);
    ~unreal();
    unrealData* simFrameRequest(float deltaTime);


};

}


#endif 