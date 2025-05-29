#ifndef MOTOR_H
#define MOTOR_H
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
    
    // Motor limits and safety
    float maxCurrent;            // Maximum safe current
    float maxTemperature;        // Maximum operating temperature
    bool isEnabled;              // Motor enable state
    
public:
    // Constructors
    motor();
    motor(float freeSpeed, float stall_torque, float stall_current, 
          float no_load_current, float motor_voltage);
    
    // Destructor
    ~motor();
    
    // Configuration methods
    void setMotorSpecs(float freeSpeed, float stall_torque, float stall_current, 
                       float no_load_current, float motor_voltage);
    void setLimits(float max_current, float max_temp);
    
    // Control methods
    void setThrottle(float throttle);           // Set throttle (0.0 - 1.0)
    void setRpm(float target_rpm);              // Set target RPM
    void enable();                              // Enable motor
    void disable();                             // Disable motor
    
    // Getter methods for specifications
    float getFreeSpeedRpm() const;
    float getStallTorque() const;
    float getStallCurrent() const;
    float getNoLoadCurrent() const;
    float getResistance() const;
    float getVoltage() const;
    float getKv() const;
    float getKt() const;
    
    // Getter methods for current state
    float getCurrentRpm() const;
    float getCurrentTorque() const;
    float getCurrentCurrent() const;
    float getCurrentThrottle() const;
    bool getIsEnabled() const;
    
    // Performance calculations
    float calculateTorqueFromCurrent(float current) const;
    float calculateCurrentFromTorque(float torque) const;
    float calculateRpmFromVoltage(float voltage) const;
    float calculatePowerOutput() const;        // Mechanical power output (W)
    float calculateEfficiency() const;         // Motor efficiency percentage
    
    // Simulation/update method
    void update(float deltaTime);              // Update motor state
};

} //SimCore
#endif // MOTOR_H