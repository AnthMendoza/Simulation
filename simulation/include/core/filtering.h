#pragma once
#include <optional>
#include <concepts>

namespace SimCore{
namespace filtering{

template<std::floating_point T>
//option to add timeout
class lowPassFilter{
private:
    T timeConstant;
    std::optional<T> maxTimeGap;
    std::optional<T> currentValue;
    
public:
    lowPassFilter(T timeConstant_, std::optional<T> maxTimeGap_ = std::nullopt):
    timeConstant(timeConstant_),maxTimeGap(maxTimeGap_),currentValue(std::nullopt){

    }
    

    T filter(T newValue, T dt){
        if(maxTimeGap.has_value() && dt >= maxTimeGap.value()){
            currentValue = newValue;
            return newValue;
        }
        
        if(!currentValue.has_value()){
            currentValue = newValue;
            return newValue;
        }

        T alpha = dt / (dt + timeConstant);
        currentValue = alpha * newValue + (1 - alpha) * currentValue.value();
        return currentValue.value();
    }
    
    void reset(){
        currentValue = std::nullopt;
    }
    
    void reset(T value){
        currentValue = value;
    }
    
    std::optional<T> getCurrentValue() const {
        return currentValue;
    }
};


template<std::floating_point T>
using threeDStateT = std::array<T, 3>;
template<std::floating_point T>
class lowPassFilterVector3{
private:
    lowPassFilter<T> filterX;
    lowPassFilter<T> filterY;
    lowPassFilter<T> filterZ;
public:
    lowPassFilterVector3(T timeConstant, std::optional<T> maxTimeGap = std::nullopt): 
        filterX(timeConstant, maxTimeGap),
        filterY(timeConstant, maxTimeGap),
        filterZ(timeConstant, maxTimeGap){

          }
    
    threeDStateT<T> filter(const threeDStateT<T>& newValue, T dt){
        threeDStateT<T> result;
        result[0] = filterX.filter(newValue[0], dt);
        result[1] = filterY.filter(newValue[1], dt);
        result[2] = filterZ.filter(newValue[2], dt);
        return result;
    }
    
    void reset(){
        filterX.reset();
        filterY.reset();
        filterZ.reset();
    }

    void reset(threeDStateT<T> vector){
        filterX.reset(vector[0]);
        filterY.reset(vector[1]);
        filterZ.reset(vector[2]);
    }
};

}
}