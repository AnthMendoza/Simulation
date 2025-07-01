#ifndef BATTERY_H
#define BATTERY_H
#include <vector>
#include <memory>
#include <string>
namespace SimCore{
class battery {
private:
    std::string configBattery;
    // Battery specifications
    float capacityAh;                    // Battery capacity in Amp-hours
    float nominalVoltage;                // Nominal voltage per cell (V)
    int cellCount;                       // Number of cells in series
    float maxVoltagePerCell;             // Maximum voltage per cell (V)
    float minVoltagePerCell;             // Minimum voltage per cell (V)
    float internalResistance;            // Internal resistance (Ohms)
    float nominalInternalResistance;
    float wattHours;                     // Energy capacity in Watt-hours
    float currentCapacity;
    // Battery state
    float soc;                           // State of charge (0 to 1)
    float voltage;                       // Current voltage (V)
    float currentDraw;                   // Current draw/charge rate (A)
    float cycleCount;                    // Number of charge/discharge cycles
    float socVoltage;
    
    // Safety and limits
    float safetyTerminationLevel;        // SOC where battery is no longer usable
    float maxDischargeCurrent;           // Maximum safe discharge current (A)

    // State tracking
    bool charged;                        // Is battery considered charged
    float timestep;                      // Time step for calculations (s)
    
    // Performance tracking
    float totalEnergyDelivered;          // Total energy delivered (Wh)
    float totalEnergyCharged;            // Total energy charged (Wh)
    float peakDischargeCurrent;          // Peak discharge current recorded
    
    // Internal methods
    float calculateVoltageFromSOC(float soc) const;
    float calculateInternalVoltageDrop(float current) const;
    void updateTemperature(float current, float deltaTime);
    bool checkSafetyLimits() const;
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
    void setBatterySpecs(float capacityAh, float nominalVoltage, int cellCount);
    void setInternalResistance(float resistance);
    void setSafetyLimits(float terminationLevel, float maxDischargeCurrent, 
                        float maxChargeCurrent, float maxTemp, float minTemp);
    void setTimestep(float timestep);
    
    // Battery state update
    void updateBattery(float current);           // Update SOC based on current draw
    // State getters
    float getSOC() const;                            // State of charge (0 to 1)
    inline float getBatVoltage() const{
        return voltage;
    }                        // Current terminal voltage
    float getRemainingCapacityAh() const;            // Remaining capacity
    float getRemainingEnergyWh() const;              // Remaining energy
    inline float getCurrentDraw() const{
        return currentDraw;
    }                    // Current draw/charge rate
    
    // Capacity and specifications
    inline float getTotalCapacityAh() const;
    inline float getTotalEnergyWh() const;

    //Nominal Cell voltage is 3.7
    inline float getNominalVoltage(){
        return 3.7 * cellCount;
    }

    inline int getCellCount() const{
        return cellCount;
    }
    inline float getInternalResistance() const {
        return internalResistance;
    }
    
    float getPowerCapability() const;  

    inline bool isCharged() const{                   // Is battery charged
        return charged;
    }

    
};
} //SimCore
#endif // BATTERY_H