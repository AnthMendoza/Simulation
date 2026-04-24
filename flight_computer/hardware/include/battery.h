#ifndef BATTERY_H
#define BATTERY_H
#include <vector>
#include <memory>
#include <string>
namespace hardware{
class battery {
private:
    std::string configBattery;
    // Battery specifications
    float totalCapacityAh,currentCapacityAh;                   // Battery capacity in Amp-hours
    float nominalVoltage;                // Nominal voltage per cell (V)
    uint8_t cellCount;                       // Number of cells in series
    float maxVoltagePerCell;             // Maximum voltage per cell (V)
    float minVoltagePerCell;             // Minimum voltage per cell (V)
    float internalResistance;            // Internal resistance (Ohms)
    float nominalInternalResistance;
    float wattHours;                     // Energy capacity in Watt-hours

    // Battery state
    uint16_t soc;                        //state of charge percentage soc = 50; => .50 on 0-1 scale
    float voltage;                       // Current voltage (V)
    float currentDraw;                   // Current draw/charge rate (A)
    float cycleCount;                    // Number of charge/discharge cycles
    float socVoltage;
    
    
    // Safety and limits
    float safetyTerminationLevel;        // SOC where battery is no longer usable
    float maxDischargeCurrent;           // Maximum safe discharge current (A)

    // State tracking
    bool charged;                        // Is battery considered charged
    float lastTimeSeconds = 0;
    bool firstSample = true;
    // Performance tracking
    float totalEnergyDelivered;          // Total energy delivered (Wh)
    float totalEnergyCharged;            // Total energy charged (Wh)
    float peakDischargeCurrent;          // Peak discharge current recorded
    
    float calculateInternalVoltageDrop(float current) const;
    // need a thermal model.
    void updateTemperature(float current, float deltaTime);
    void updateVoltage(float current);
    
public:
    // Constructors
    battery(std::string& config);
    //battery(float capacityAh, float nominalVoltage, int cellCount, float initialSoc = 1.0f);
    void init(std::string& config);

    // Destructor
    ~battery() = default;


    
    // Configuration methods
    inline void setSOC(float level){
        soc = level;
    }
    void setBatterySpecs(float capacityAh, float voltage, int cellCount);
    void setInternalResistance(float resistance);
    void setSafetyLimits(float terminationLevel, float maxDischargeCurrent,float maxChargeCurrent = 10.0f, float maxTemp = 100.0f, float minTemp = 0.0f);
    
    // Battery state update
    void updateBattery(float current , float currentTimeSeconds); // Update SOC based on current draw

    //voltage in volts
    void voltageSensorInput(float sensorVoltageV){
        voltage = sensorVoltageV;
    }

    // State getters
    float getSOC() const{
        return soc;
    };                            // State of charge (0 to 1)
    inline float getBatVoltage() const{
        return voltage;
    }                        // Current terminal voltage
    float getRemainingCapacityAh() const;            // Remaining capacity
    float getRemainingEnergyWh() const{
        return totalCapacityAh* getNominalVoltage();
    }              // Remaining energy
    inline float getCurrentDraw() const{
        return currentDraw;
    }                    // Current draw/charge rate
    
    // Capacity and specifications
    inline float getTotalCapacityAh() const{
        return totalCapacityAh;
    }
    inline float getTotalEnergyWh() const{
        return wattHours;
    }

    //Nominal Cell voltage is 3.7
    inline float getNominalVoltage() const{
        return 3.7 * cellCount;
    }

    inline float getVoltage() const{
        return voltage;
    }

    inline int getCellCount() const{
        return cellCount;
    }
    inline float getInternalResistance() const {
        return internalResistance;
    }
    
    float getPowerCapability() const;  

    inline bool isCharged() const{// Is battery charged
        return charged;
    }

    inline float getSafetyTerminationLevel(){
        return safetyTerminationLevel;
    }

    bool manuallySetInitVoltage(float voltageV);

    
};
} //SimCore
#endif // BATTERY_H