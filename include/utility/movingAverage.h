#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H


#include <vector>
#include <stdexcept>
#include <cmath>

namespace utility{

template<typename T>
class movingAverage {
private:
    std::vector<T> buffer;
    size_t windowSize;
    size_t currentIndex;
    size_t count;
    T sum;
    T sumSquares;
    bool isFull;

public:
    movingAverage(size_t windowSize) 
        : windowSize(windowSize), currentIndex(0), count(0), sum(0), sumSquares(0), isFull(false) {
        if (windowSize <= 0) {
            throw std::invalid_argument("Window size must be greater than 0");
        }
        buffer.resize(windowSize);
    }

    void add(const T& value) {
        if (isFull) {
            T oldValue = buffer[currentIndex];
            sum -= oldValue;
            sumSquares -= oldValue * oldValue;
        }
        buffer[currentIndex] = value;
        sum += value;
        sumSquares += value * value;
        
        currentIndex = (currentIndex + 1) % windowSize;
        
        if (count < windowSize) {
            count++;
        } else {
            isFull = true;
        }
    }

    T getAverage() const {
        if (count == 0) {
            throw std::runtime_error("No values added yet");
        }
        return sum / static_cast<T>(count);
    }

    T getVariance() const {
        if (count == 0) {
            throw std::runtime_error("No values added yet");
        }
        if (count == 1) {
            return 0;
        }
        
        T mean = getAverage();
        T variance = (sumSquares / static_cast<T>(count)) - (mean * mean);
        return variance;
    }

    T getStandardDeviation() const {
        return std::sqrt(getVariance());
    }

    T getSampleVariance() const {
        if (count <= 1) {
            throw std::runtime_error("Sample variance requires at least 2 values");
        }
        
        T mean = getAverage();
        T variance = (sumSquares - static_cast<T>(count) * mean * mean) / static_cast<T>(count - 1);
        return variance;
    }

    T getSampleStandardDeviation() const {
        return std::sqrt(getSampleVariance());
    }

    void reset() {
        currentIndex = 0;
        count = 0;
        sum = 0;
        sumSquares = 0;
        isFull = false;
    }

    size_t size() const {
        return count;
    }

    size_t capacity() const {
        return windowSize;
    }

    bool full() const {
        return isFull;
    }
};

}

#endif // MOVING_AVERAGE_H