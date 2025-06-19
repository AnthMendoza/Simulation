#include "../include/droneControl.h"
#include <iostream>
#include "../include/Eigen/Eigen"
#include "../include/Eigen/Dense"
#include <vector>

using namespace Eigen;
using namespace std;
namespace SimCore{

void controlAllocator::buildCASMatrix() {
    int n = positions.size();  // Number of motors
    B.resize(6, n);            // CAS is 6 x N (3 force + 3 torque)
    for (int i = 0; i < n; ++i){
        const Vector3d& r = positions[i];     // Motor position
        const Vector3d& f = thrustDirs[i];   // Thrust direction (usually +Z)
        double t = spinTorque(i);            // Spin torque coefficient
        Vector3d force = f;                    // Force contribution (assumed unit thrust)
        //linear torque curve. not ideal but somewhat useful
        Vector3d torque = r.cross(f) + t * f;  // Total torque: position × thrust + spin-induced torque
        // Assign to CAS matrix: first 3 rows = force, last 3 = torque
        B.block<3,1>(0, i) = force;
        B.block<3,1>(3, i) = torque;
    }
    // Compute Moore-Penrose pseudo-inverse for solving under/over-determined systems
    JacobiSVD<MatrixXd> svd(B, ComputeThinU | ComputeThinV);
    Bpinv = svd.solve(MatrixXd::Identity(6, 6));
}

VectorXd controlAllocator::toVectorXd(std::initializer_list<float> list) {
    VectorXd vec(list.size());
    int i = 0;
    for (float  val : list) vec(i++) = val;
    return vec;
}

controlAllocator::controlAllocator(const vector<array<float,3>>& motorPositions,const vector<array<float,3>>& thrustDirections,vector<float> spinTCoefficent){   
    if(motorPositions.empty() || thrustDirections.empty() || spinTCoefficent.empty()) throw runtime_error("Allocator contains no motor positions");
    //converts native c++ into eigan friendly Vectors.
    //check if vectors are the same size. Throws runtime error if not.
    if(motorPositions.size() != thrustDirections.size() || motorPositions.size() != spinTCoefficent.size()){
        throw runtime_error("Vector values in constructor are not the same size. 4 motors needs 4 thrust directions");
    }
    //VectorXd is initalized to 0 and the size needs to be set much like a static array.
    //VectorXd += value does not add slots
    spinTorque.resize(motorPositions.size());// n = number of motors
    for(int i = 0 ; i < motorPositions.size();i++){
        const auto& pos = motorPositions[i];
        positions.push_back(Vector3d(static_cast<double>(pos[0]), static_cast<double>(pos[1]), static_cast<double>(pos[2])));

        const auto& dir = thrustDirections[i];
        thrustDirs.push_back(Vector3d(static_cast<double>(dir[0]), static_cast<double>(dir[1]), static_cast<double>(dir[2])));

        // equivlent to .push_back for a VectorXd
        spinTorque(i) += spinTCoefficent[i];
    }
    buildCASMatrix();  // Build CAS matrix and precompute pseudo-inverse
}



} //SimCore
