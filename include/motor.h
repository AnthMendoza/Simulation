#ifndef MOTOR_H
#define MOTOR_H
#include <string>

namespace SimCore{
class motor {
private:
    // Motor specifications
    float freeSpeedRpm;          // Maximum RPM at no load
    float stallTorque;           // Maximum torque at zero speed (Nm)
    float stallCurrent;          // Current draw at stall (A)
    float noLoadCurrent;         // Current draw at no load (A)
    float resistance;            // Motor resistance (Ohms)
    float voltage;               // Operating voltage (V)
    float kv;                    // Motor velocity constant (RPM/V)
    float kt;                    // Motor torque constant (Nm/A)
    
    // Current operating state
    float currentRpm;            // Current motor speed
    float currentTorque;         // Current torque output
    float currentCurrent;        // Current draw
    float currentThrottle;       // Throttle input (0.0 - 1.0)
    float maxCurrentAvailable;
    
    // Motor limits and safety
    bool isEnabled;              // Motor enable state
    
public:
    // Constructors
    motor(std::string& config);
    //motor(float freeSpeed, float stall_torque, float stall_current, float no_load_current, float motor_voltage);
    void init(std::string& motorConfig);
    // Destructor
    ~motor() = default;
    
    // Configuration methods
    void setMotorSpecs(float freeSpeed, float stall_torque, float stall_current, 
                       float no_load_current, float motor_voltage);
    void setLimits(float max_current, float max_temp);
    
    // Control methods
    void rpmRequest(float rpm);
    void enable();                              // Enable motor
    void disable();                             // Disable motor
     void setCurrent(float current);
    

    // Getter methods for specifications
    inline float getFreeSpeedRpm(){
        return freeSpeedRpm;
    }
    inline float getStallTorque() const{
        return stallTorque;
    }
    inline float getStallCurrent() const{
        return stallCurrent;
    }
    inline float getNoLoadCurrent() const{
        return noLoadCurrent;
    }
    inline float getVoltage() const{
        return voltage;
    }
    inline float getKv() const{
        return kv;
    }
    inline float getKt() const{
        return kt;
    }
    
    
    float getCurrentRpm() const;
    float getCurrentTorque() const;
    inline float getCurrentCurrent() const{
        return currentCurrent;
    }
    float getCurrentThrottle() const;
    bool getIsEnabled() const;
    
    // Performance calculations
    float calculateTorqueFromCurrent(float current) const;
    float calculateCurrentFromTorque(float torque) const;
    float calculateRpmFromVoltage(float voltage);
    inline float calculatePowerOutput(){
        return voltage * getCurrentCurrent();
    }
    float calculateEfficiency() const;         // Motor efficiency percentage
    
    // Simulation/update method
    void update();              // Update motor state
};

} //SimCore
#endif // MOTOR_H