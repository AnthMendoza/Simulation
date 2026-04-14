#pragma once

namespace utility{
    
    template<typename T>
    T hz_sec(T sec){
        if(sec == 0) return 0;
        return 1/sec;
    }

} // namespace utility
