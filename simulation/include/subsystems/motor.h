#ifndef MOTOR_H
#define MOTOR_H
#include <string>
#include "../control/PIDController.h"
#include "battery.h"

namespace SimCore{
class motor {
private:
    // Motor specifications
    float freeSpeedAngularVelocity;          // Maximum RPM at no load
    float stallTorque;           // Maximum torque at zero speed (Nm)
    float currentLimit;          // speed controller current limit(A)
    float noLoadCurrent;         // Current draw at no load (A)
    float coilResistance;            // Motor resistance (Ohms)
    float voltage;               // Operating voltage (V)
    float kv;                    // Motor velocity constant (V/rad/s)
    float kt;                    // Motor torque constant (Nm/A)
    float ke;                    // ke = kt for ideal motor
    float dampingCoeff;
    float netTorque;             // equals motorTorque - loads
    float motorTorque;
    
    // Current operating state
    float currentAngularVelocity;            // Current motor speed
    float currentTorque;         // Current torque output
    float currentCurrent;        // Current draw
    float currentThrottle;       // Throttle input (0.0 - 1.0)
    float maxCurrentAvailable;
    float maxVoltage;
    float appliedVoltage;
    float inertia;               // kg·m² (rotor inertia)
    float angularVeloRequest;
    float backEMF;
    float electricalPower;
    float mechanicalPower;
    float efficiency;
    // Motor limits and safety
    bool isEnabled;              // Motor enable state

    const float MAX_ANGULAR_ACCEL = 12000.0f; // rad/s²
    
    std::unique_ptr<PIDController> PID;
public:
    /**
     * @brief Updates the motor's electrical and mechanical state over a time step.
     *
     * Sources: https://www.ece.rice.edu/~jdw/435/book/ch8?utm_source=chatgpt.com
     *
     * @param timeStep The time increment for the simulation step (in seconds).
     * @param motorResistance The mechanical torque opposing the motor's rotation (in Nm).
     * @param coilResistance The electrical resistance of the motor coils (in ohms).
     */
    void updateMotor(float timeStep , float loadTorque , float voltage);
    void updateMotorAngularVelocity(float timeStep , float loadTroque , battery& bat, float rad_sec);
    /// @brief voltage applied to motor  = state * batteryVoltage.
    /// @param state value -1 to 1;
    // Constructors
    motor() = delete;
    motor(std::string& config); 
    motor(const motor& other);
    //motor(float freeSpeed, float stall_torque, float stall_current, float no_load_current, float motor_voltage);
    void init(std::string& motorConfig);
    // Destructor
    ~motor();
    
    // Configuration methods
    void setMotorSpecs(float freeSpeed, float stall_torque, float stall_current, 
                       float no_load_current, float motor_voltage);
    void setLimits(float max_current, float max_temp);

    inline void setMotorAngularVelocity(float rad_sec){
        currentAngularVelocity = rad_sec;
    }
    
    // Control methods
    void angularVeloctiyRequest(float rad_per_sec);
    void enable();                              // Enable motor
    void disable();                             // Disable motor
    void setCurrent(float current);
    

    // Getter methods for specifications
    inline float getFreeSpeedAngularVelocity(){
        return freeSpeedAngularVelocity;
    }
    inline float getStallTorque() const{
        return stallTorque;
    }
    inline float getCurrentLimit() const{
        return currentLimit;
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
    inline float getAppliedVoltage(){
        return appliedVoltage;
    }
    inline float getAngularVelocityRequest(){
        return angularVeloRequest;
    }

    
    /**
     * @brief Motors current rotational Velocity
     * @return rad/s
     */
    inline float getCurrentAngularVelocity() const{
        return currentAngularVelocity;
    }
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

    inline void setVoltage(float volt){
       volt = std::clamp(volt,-maxVoltage,maxVoltage);
       appliedVoltage = volt;
    }

    inline void resetMotor() {
        currentAngularVelocity = 0.0f;
        currentTorque = 0.0f;
        currentCurrent = 0.0f;
        currentThrottle = 0.0f;
        appliedVoltage = 0.0f;
        angularVeloRequest = 0.0f;

        netTorque = 0.0f;
        motorTorque = 0.0f;
        backEMF = 0.0f;
        electricalPower = 0.0f;
        mechanicalPower = 0.0f;
        efficiency = 0.0f;
        //reset PID to prevent parasitic integral wind up 
        if (PID) {
            PID->reset();
        }
    }
    
    
};

} //SimCore
#endif // MOTOR_H