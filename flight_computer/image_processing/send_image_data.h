
#include "../third_party/async-sockets/include/udpsocket.hpp"
#include "image_encoding.h"
#include <iostream>
#include <string>
using namespace std;

class send_image_data {
private:
    UDPSocket<> udpClient;
    string target_ip;
    uint16_t target_port;
    
public:
    send_image_data(string ip, uint16_t port) : target_ip(ip), target_port(port) {
        udpClient.Connect(ip, port, [](int errorCode, string errorMessage) {
            cout << "Connect: " << errorCode << " : " << errorMessage << endl;
        });
    }
    


    void send_image(image_processing::image_encoding& img) {
        std::cout<<img.header_size() << std::endl;
        udpClient.Send(reinterpret_cast<const char*>(img.header_data()), img.header_size() );

        std::cout<<img.pixel_data_size()<< std::endl;
        udpClient.Send(reinterpret_cast<const char*>(img.raw_pixel_data()) , img.pixel_data_size());

    }
        
 

};
