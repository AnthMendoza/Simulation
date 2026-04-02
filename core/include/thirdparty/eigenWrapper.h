#pragma once
//the wrapped is needed due to some nasty collisions with unreal macros
#if defined(UE_BUILD_DEBUG) || defined(UE_BUILD_DEVELOPMENT) || defined(UE_BUILD_SHIPPING) || defined(UE_BUILD_TEST)
    #define USING_UNREAL_ENGINE
#endif

#ifdef USING_UNREAL_ENGINE
    #pragma push_macro("check")
    #pragma push_macro("ensure")
    
    #undef check
    #undef ensure
#endif

#include <Eigen/Eigen>
#include <Eigen/Dense>


#ifdef USING_UNREAL_ENGINE
    #pragma pop_macro("check")
    #pragma pop_macro("ensure")
#endif