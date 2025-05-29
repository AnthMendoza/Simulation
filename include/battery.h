#ifndef BATTERY_H
#define BATTERY_H
namespace SimCore{
class battery {
private:
    // Battery specifications
    float capacityAh;                    // Battery capacity in Amp-hours
    float nominalVoltage;                // Nominal voltage per cell (V)
    int cellCount;                       // Number of cells in series
    float maxVoltagePerCell;             // Maximum voltage per cell (V)
    float minVoltagePerCell;             // Minimum voltage per cell (V)
    float internalResistance;            // Internal resistance (Ohms)
    float wattHours;                     // Energy capacity in Watt-hours
    
    // Battery state
    float soc;                           // State of charge (0 to 1)
    float voltage;                       // Current voltage (V)
    float currentDraw;                   // Current draw/charge rate (A)
    float cycleCount;                    // Number of charge/discharge cycles
    
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
    void updateVoltage();
    
public:
    // Constructors
    battery();
    battery(float capacityAh, float initialSoc = 1.0f);
    battery(float capacityAh, float nominalVoltage, int cellCount, float initialSoc = 1.0f);
    void init();

    // Destructor
    ~battery();


    
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
    void updateBattery(float currentAmps);           // Update SOC based on current draw

    // State getters
    float getSOC() const;                            // State of charge (0 to 1)
    float getVoltage() const;                        // Current terminal voltage
    float getRemainingCapacityAh() const;            // Remaining capacity
    float getRemainingEnergyWh() const;              // Remaining energy
    float getCurrentDraw() const;                    // Current draw/charge rate
    
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