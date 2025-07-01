#include "../dynamics/drone.h"

namespace SimCore{

class calibratePID {
private:

    float alpha, beta;
    float costFunctionSum = 0.0f;
    float tolerance = 0.02f;

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
    calibratePID(float alpha, float beta)
        : alpha(alpha), beta(beta) {}

    void reset() {
        costFunctionSum = 0.0f;
        timeLog.clear();
        outputLog.clear();
        referenceLog.clear();
    }

    void logStep(float t, float output, float reference , float dt) {
        float error = reference - output;
        costFunction(error , dt);
        timeLog.push_back(t);
        outputLog.push_back(output);
        referenceLog.push_back(reference);
    }

    float evaluate() {
        float finalReference = referenceLog.empty() ? 0.0f : referenceLog.back();
        float overshoot = computeOvershoot(outputLog, finalReference);
        float settlingTime = computeSettlingTime(outputLog, finalReference, timeLog);

        return costFunctionSum + alpha * overshoot + beta * settlingTime;
    }
};



}