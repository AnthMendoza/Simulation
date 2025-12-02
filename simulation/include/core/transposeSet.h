#pragma once
#include <vector>
#include "transpose.h"

namespace SimCore{

class transposeSet{
private:
    std::vector<transpose*> entities;

public:
    template<class t>
    void insertObject(t& obj){
        if (auto* basePtr = dynamic_cast<transpose*>(&obj)) {
            entities.push_back(basePtr);
        }
    }

    void rotate(const Quaternion& quant ,const poseState& poseOfIntrest , const poseState& reference = CoordinateSystem::WORLD_BASIS){
        for(auto& entity:entities){
            entity->transposeSelf(quant , poseOfIntrest , reference);
        }
    }

};

}