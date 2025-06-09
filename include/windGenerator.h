#ifndef WINDGENERATOR_H
#define WINDGENERATOR_H 

#include <random>
class turbulence{
public:
    turbulence(float alpha = 0.95f, float mean = 0.0f, float stddev = 3.0f)
        : alpha(alpha), mean(mean), stddev(stddev), previous(0.0f){
        std::random_device rd;
        rng = std::mt19937(rd());
        dist = std::normal_distribution<float>(mean, stddev);
    }
    // returns next stochastic value. Filtered through a low pass filter.
    float getNext() {
        float whiteNoise = dist(rng);
        float filtered = alpha * previous + (1.0f - alpha) * whiteNoise;
        previous = filtered;
        return filtered;
    }

    void reset(float value = 0.0f) {
        previous = value;
    }

    void setAlpha(float a) {
        alpha = a;
    }

    void setMean(float m) {
        mean = m;
        dist = std::normal_distribution<float>(mean, stddev);
    }

    void setStdDev(float s) {
        stddev = s;
        dist = std::normal_distribution<float>(mean, stddev);
    }

private:
    float alpha;
    float mean;
    float stddev;
    float previous;

    std::mt19937 rng;
    std::normal_distribution<float> dist;
};


#endif
