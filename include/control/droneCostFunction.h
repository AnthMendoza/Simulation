#include <vector>

#pragma once

namespace SimCore{


/**
 * @class calibratePID
 * @brief Utility class for evaluating and tuning PID controller performance.
 *
 * This class provides a generalized framework for calculating the cost of a given set of PID gains
 * based on simulation results.
 *
 * Usage:
 *  - Call reset() before each simulation run.
 *  - During each timestep, call logStep(time, output, reference).
 *  - After the run, call evaluate() to get the total cost.
 *
 * Source paper: https://skoge.folk.ntnu.no/prost/proceedings/PID-12/papers/0030.pdf?utm_source=chatgpt.c
 * 
 * Cost function highlighted in the paper is slightly simplified due to open loop requests. Open loop
 * implies that the tested parameter has feedforward control.
 */

class calibratePID {
private:

    float alpha = 0.0f;
    float beta = 0.0f;
    float costFunctionSum = 0.0f;
    float tolerance = 0.02f;
    float currentTime = 0;
    std::vector<float> timeLog;
    std::vector<float> outputLog;
    std::vector<float> referenceLog;

    void costFunction(float error ,float dt) {
        costFunctionSum += (error * error) * dt;
    }

    float computeOvershoot(const std::vector<float>& output, float reference) {
        float maxValue = *std::max_element(output.begin(), output.end());
        return std::max(0.0f, maxValue - reference);
    }

    float computeSettlingTime(const std::vector<float>& output, float reference, const std::vector<float>& time) {
        float threshold = std::abs(reference * tolerance);
        for (int i = output.size() - 1; i >= 0; --i) {
            if (std::abs(output[i] - reference) > threshold)
                return time[i];
        }
        return 0.0f;
    }

public:
    calibratePID(){

    }

    void reset() {
        costFunctionSum = 0.0f;
        currentTime = 0;
        timeLog.clear();
        outputLog.clear();
        referenceLog.clear();
    }

    void logStep(float output, float reference , float dt) {
        currentTime += dt;
        float error = reference - output;
        costFunction(error , dt);
        timeLog.push_back(currentTime);
        outputLog.push_back(output);
        referenceLog.push_back(reference);
    }

    float evaluate() {
        float finalReference = referenceLog.empty() ? 0.0f : referenceLog.back();
        float overshoot = computeOvershoot(outputLog, finalReference);
        float settlingTime = computeSettlingTime(outputLog, finalReference, timeLog);
        costFunctionSum = costFunctionSum + (alpha * overshoot + beta * settlingTime);
        return costFunctionSum;
    }
    
    inline void setConstants(float _alpha, float _beta){
        alpha = _alpha;
        beta = _beta;
    }
};

}