#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H


struct pid_con {
    float kp{0.0f};
    float ki{0.0f};
    float kd{0.0f};       
    float i_limit{0.0f};     
    float output_limit{0.0f};
};


class PIDController {
public:
    PIDController() = delete;
    PIDController(float kp, float ki, float kd);
    PIDController(float kp, float ki, float kd, float min, float max, float i_limit);
    PIDController(const pid_con& config);
    PIDController(const PIDController& other);

    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    void reset();
    void setTarget(float t);
    void setIntegralClamp(float clamp);
    float update(const float& measurement,const float deltaTime);

    inline float getKp() const { return kp; }
    inline float getKi() const { return ki; }
    inline float getKd() const { return kd; }

    inline float getTarget() const { return target; }

    inline float getIntegral() const { return integral; }
    inline float getPreviousError() const { return previousError; }

    inline float getMinOutput() const { return minOutput; }
    inline float getMaxOutput() const { return maxOutput; }
    inline float lastError() const {return target - previousSample;}
    inline float getPreviousSample() const {return previousSample;}
    inline float getPreviousOutput() const {return previousOutput;}

private:
    float kp;
    float ki;
    float kd;

    float target;

    float integral; //sum of errors
    float clampInt;
    float previousError;  
    float previousSample;
    float previousOutput;
    float minOutput;
    float maxOutput;
};

#endif // PIDCONTROLLER_H
