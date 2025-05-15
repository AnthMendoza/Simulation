#include "../include/unreal.h"

#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include "../include/control.h"

#define SERVER_IP "127.0.0.1"
#define SERVER_PORT 8000


unreal::unreal(): unrealVehicle() , packet(){
    updateFrequency = 120;
}


void unreal::sendUDP() {
    udpState = true; 
    int sockfd;
    struct sockaddr_in serverAddr;
    
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
      perror("Socket creation failed");
      exit(EXIT_FAILURE);
    }
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SERVER_PORT);
    serverAddr.sin_addr.s_addr = inet_addr(SERVER_IP);
    std::chrono::time_point<std::chrono::system_clock> start,end;
    start = std::chrono::system_clock::now();
    unrealData dataPacket;
    while (udpState) {
        getPacket(dataPacket);
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed  = end - start;
        std::cout<< "UDP time:" << elapsed.count() << " Sim Time:"<< unrealVehicle.getTime() << " Error:" << unrealVehicle.getTime() - elapsed.count()<< "\n";
        char message[sizeof(unrealData)];
        std::memcpy(message, &dataPacket, sizeof(unrealData));
        int sent_bytes = sendto(sockfd, message, sizeof(unrealData), 0 ,(struct sockaddr *)&serverAddr, sizeof(serverAddr));
        if (sent_bytes < 0) {
          perror("Failed to send packet");
        }
        usleep(1/updateFrequency * 1000000.0f); 
    }
  
    close(sockfd);
  }
  


bool unreal::setPacket(float time){
    std::lock_guard<std::mutex> lock(packetMutex);
    packet.timeStamp = time;
    auto velo = unrealVehicle.getVelocityVector();
    packet.velocity[0] = velo[0];
    packet.velocity[1] = velo[1];
    packet.velocity[2] = velo[2];
    auto pos =  unrealVehicle.getPositionVector();
    packet.position[0] = pos[0];
    packet.position[1] = pos[1];
    packet.position[2] = pos[2];
    auto state = getState();
    packet.rotation[0] = state[0];
    packet.rotation[1] = state[1];
    packet.rotation[2] = state[2]; 

    return true;
}

void unreal::getPacket(unrealData &dataPacket){
    std::lock_guard<std::mutex> lock(packetMutex);
    dataPacket = packet;
}

void unreal::iterator(){
    std::chrono::time_point<std::chrono::system_clock> start,end;
    start = std::chrono::system_clock::now();
    while(true){
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        if(unrealVehicle.getPositionVector()[2]> 0 && unrealVehicle.getTime() <=  elapsed.count()){
            unrealVehicle.drag();
            unrealVehicle.lift();
            unrealVehicle.reentryBurn();
            unrealVehicle.landingBurn();
            //data->logRocketPosition(rocket); //future simulate in unreal and display logs. not logging for now
            unrealVehicle.updateState();
            int iteration = unrealVehicle.getIterations();
            setIterations(iteration++);
        }
    }
}

unreal::~unreal() = default;

int main(){
    unreal *unrealVehicle = new unreal;
    std::thread udp(&unreal::sendUDP,unrealVehicle); 
    std::thread iterator(&unreal::iterator , unrealVehicle);
    while(true){}
    delete unrealVehicle;
    
    return 0;
}

