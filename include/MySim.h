#ifndef MYSIM_H
#define MYSIM_H
#include "vehicle.h"
#include "rocket.h"
#include <memory>
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
    std::string configFile;
    void setPacket();
    void getPacket(unrealData &dataPacket);
    void iterator(float totalTime);
    float totalTime;
    public:
    std::unique_ptr<Rocket> unrealVehicle;
    unreal(const std::string tomlData);
    ~unreal();
    unrealData* simFrameRequest(float deltaTime);
};

}


#endif 