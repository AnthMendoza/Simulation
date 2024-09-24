#include <iostream>




class vehicleState{
    public:

    void display() const {
    std::cout << "Position: (" << Xposition << ", " << Yposition << ", " << Zposition << ")\n"
              << "Orientation (Roll, Pitch, Yaw): (" << roll << ", " << pitch << ", " << yaw << ")\n";
    }

    private:
    
        float   Xposition , Yposition , Zposition;     // position
        float    roll , pitch, yaw ;    // rotation

};