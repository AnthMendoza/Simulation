#include <zlib.h>
namespace com_utils {
    template<class P>
    //ensure packet crc32 is named crc32 the function exludes itin the crc calculation.
    //due to the trival resizing of the crc field uint32_t crc32 MUST be last in the packet.
    void calculate_crc(P& packet) {
        packet.crc32 = 0;
        size_t header_size = reinterpret_cast<const uint8_t*>(&packet.crc32) - 
                           reinterpret_cast<const uint8_t*>(&packet);
        size_t total_size = header_size + packet.payload_length;
        
        packet.crc32 = crc32(0L, reinterpret_cast<const Bytef*>(&packet), total_size);
    }
    
    template<class P>
    bool verify_crc(const P& packet) {
        P temp = packet;
        temp.crc32 = 0;
        
        size_t header_size = reinterpret_cast<const uint8_t*>(&temp.crc32) - 
                           reinterpret_cast<const uint8_t*>(&temp);
        size_t total_size = header_size + packet.payload_length;
        
        uint32_t calculated = crc32(0L, reinterpret_cast<const Bytef*>(&temp), total_size);
        return calculated == packet.crc32;
    }
}