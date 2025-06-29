#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H




class PIDController {
public:
    PIDController() = delete;
    PIDController(float kp, float ki, float kd, float dt);
    PIDController(const PIDController& other);

    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    void reset();
    void setTimeStep(float timeStep);
    void setTarget(float t);
    float update(const float& measurement);

    inline void clampIntegral(int value = 100){
        clampInt = value;
    }

    inline float getKp() const { return kp; }
    inline float getKi() const { return ki; }
    inline float getKd() const { return kd; }
    inline float getDt() const { return dt; }

    inline float getTarget() const { return target; }

    inline float getIntegral() const { return integral; }
    inline float getPreviousError() const { return previousError; }

    inline float getMinOutput() const { return minOutput; }
    inline float getMaxOutput() const { return maxOutput; }
    inline float lastError() const {return target - previousSample;}
    inline float getPreviousSample() const {return previousSample;}
    

private:
    float kp;
    float ki;
    float kd;
    float dt;

    float target;

    float integral; //sum of errors
    float clampInt;
    float previousError;  
    float previousSample;
    float minOutput;
    float maxOutput;
};

#endif // PIDCONTROLLER_H
