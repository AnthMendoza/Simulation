#include <vector>
#include  <iostream>
#include <cmath>


void lowPrecisionData(std::vector<float> &data , std::vector<float> &returnData ,  int desiredResolution){
    if(desiredResolution <= 0) throw std::invalid_argument ("desiredResolution cannot be zero");

    float ratio  = data.size()/desiredResolution;
    int roundedRatio = std::floor(ratio);
    if(roundedRatio < 2){
        returnData = data;
        return;
    }

    for(int i = 0 ; i < data.size() ; i++){
        if( i % roundedRatio == 0 ){
            returnData.push_back(data[i]);
        }
    }

}

int main(){

    std::vector<float> nums;

    for(int i = 0 ; i<10000 ; i++){

        nums.push_back(i);

    }   
    std::vector<float> returnData;
    int num = 100;
    lowPrecisionData(nums , returnData  ,  num);

    for(int i = 0 ; i < returnData.size() ; i++){
        std::cout<< returnData[i]<< std::endl;
    }


    return 0;
}