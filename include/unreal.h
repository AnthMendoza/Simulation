#ifndef UNREAL_H
#define UNREAL_H
#include "vehicle.h"
#include <mutex>
#pragma pack(push,1)
struct unrealData{
    float timeStamp;
    float position[3];
    float velocity[3];
    float rotation[3]; //format undecided
};
#pragma pack(pop)


class unreal: public Vehicle{
    private:
    unrealData packet;
    Rocket unrealVehicle;
    float updateFrequency;
    bool udpState;
    std::mutex packetMutex;
    bool setPacket(float time);
    void getPacket(unrealData &dataPacket);
    public:
    void sendUDP();
    unreal();
    ~unreal();
    void iterator();


};




#endif 