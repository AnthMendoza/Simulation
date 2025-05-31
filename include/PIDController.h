#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float dt);
    PIDController() = delete;
    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float minOutput, float maxOutput);
    void reset();
    void setTimeStep(float timeStep);
    void setTarget(float t);
    float update(float measurement);

private:
    float kp;
    float ki;
    float kd;
    float dt;

    float target;

    float integral; //sum of errors
    float previousError;

    float minOutput;
    float maxOutput;
};

#endif // PIDCONTROLLER_H
