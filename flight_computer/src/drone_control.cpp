#include <drone_control.h>
#include <vector>

using namespace Eigen;
using namespace std;
namespace avionics{

void controlAllocator::buildCASMatrix() {
    int n = positions.size();  // Number of motors
    B.resize(6, n);            // CAS is 6 x N (3 force + 3 torque)
    for (int i = 0; i < n; ++i){
        const Vector3d& r = positions[i];     // Motor position
        const Vector3d& f = thrustDirs[i];   // Thrust direction (usually +Z)
        double t = spinTorque(i);            // Spin torque coefficient
        Vector3d force = f;                    // Force contribution (assumed unit thrust)
        
        Vector3d torque = r.cross(f) + t * f;  
        
        B.block<3,1>(0, i) = force;
        B.block<3,1>(3, i) = torque;
    }
    // Compute Moore-Penrose pseudo-inverse for solving under/over-determined systems
    JacobiSVD<MatrixXd> svd(B, ComputeThinU | ComputeThinV);
    Bpinv = svd.solve(MatrixXd::Identity(6, 6));
}

VectorXd controlAllocator::toVectorXd(std::initializer_list<units::scalar> list) {
    VectorXd vec(list.size());
    int i = 0;
    for (units::scalar  val : list) vec(i++) = val;
    return vec;
}

controlAllocator::controlAllocator(const vector<array<units::scalar,3>>& motorPositions,const vector<array<units::scalar,3>>& thrustDirections,vector<units::scalar> spinTCoefficient){   
    if(motorPositions.empty() || thrustDirections.empty() || spinTCoefficient.empty()){
        spdlog::critical("Allocator contains no actuators.");
    }

    if(motorPositions.size() != thrustDirections.size() || motorPositions.size() != spinTCoefficient.size()){
        spdlog::critical("Vector values in constructor are not the same size. 4 motors needs 4 thrust directions");
    }

    spinTorque.resize(motorPositions.size());
    for(int i = 0 ; i < motorPositions.size();i++){
        const auto& pos = motorPositions[i];
        positions.push_back(Vector3d(static_cast<double>(pos[0]), static_cast<double>(pos[1]), static_cast<double>(pos[2])));

        const auto& dir = thrustDirections[i];
        thrustDirs.push_back(Vector3d(static_cast<double>(dir[0]), static_cast<double>(dir[1]), static_cast<double>(dir[2])));

        spinTorque(i) += spinTCoefficient[i];
    }
    buildCASMatrix(); 
}


allocatorData controlAllocator::computeAllocation(controlPacks::forceMoments requestPacket) {
    allocatorData result;

    auto force = requestPacket.force;
    auto moments = requestPacket.moments;

    Eigen::VectorXd desired = toVectorXd({force[0],force[1],force[2],moments[0],moments[1],moments[2]});

    Eigen::VectorXd thrusts = allocate(desired);

    Eigen::VectorXd output = computeWrench(thrusts);
    

    
    for (int i = 0; i < thrusts.size(); ++i) {
        result.thrusts.push_back(static_cast<units::scalar>(thrusts[i]));
    }

    for (int i = 0; i < 3; ++i) {
        result.forces[i] = static_cast<units::scalar>(output[i]);
    }

    for (int i = 0; i < 3; ++i) {
        result.moments[i] = static_cast<units::scalar>(output[i + 3]);
    }

    manageMovingAvg(result);

    return result;
}


void controlAllocator::manageMovingAvg(allocatorData& allocation) {
    const size_t windowsSize = 3;
    if (movingAvg.size() < allocation.thrusts.size()) {
        while (movingAvg.size() < allocation.thrusts.size()) {
            utility::movingAverage<units::scalar> obj(windowsSize);
            movingAvg.push_back(obj);
        }
    }else if (movingAvg.size() > allocation.thrusts.size()) {
        while (movingAvg.size() > allocation.thrusts.size()) {
            movingAvg.pop_back();
        }
    }

    for(size_t i = 0 ; i < allocation.thrusts.size() ; i++){
        auto& avg = movingAvg[i];
        avg.add(allocation.thrusts[i]);
        allocation.thrusts[i] = avg.getAverage();
    }
    
}


} //SimCore
