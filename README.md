# Flight Computer Project

> **Status: Work In Progress** — This project is under active development. 

## Overview
This project implements a modular **flight computer**, **ground station**, and **simulation**. **Missions** characterize high level behaviors. It is designed with performance and flexibility in mind, using modern C++ patterns for real-time or near-real-time systems.

A key focus of this project is high-fidelity simulation, including realistic sensor noise modeling, enabling more accurate testing of estimation and control systems before deployment.

---

## Features

* Control system (PID + feedforward / cascaded controllers)
* State estimation, Kalman Filter (work in progress, [Kalman source documentation](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python))
* Simulation environment for testing with realistic sensor noise injection
* Telemetry via UDP viewable from a PyQt6 GUI
* Logging (spdlog integration)
* Configurable parameters (Transitioning from TOML to [YAML-cpp](https://github.com/jbeder/yaml-cpp))

---

## Project Structure

```
root
├── core/
├── flight_computer/
├── ground_station/
├── missions/
└── simulation/
```
---
## Getting Started

### Prerequisites

* C++20 or newer
* CMake (>= 3.16)
* Compiler (GCC / Clang)
---

### Build

```bash
git clone https://github.com/AnthMendoza/Simulation.git
cd Simulation
mkdir build && cd build
cmake ..
make -j
```

---

### Run 

```bash
./missions/<mission_name>/<mission_executable>
```

---

## Configuration

Configuration is handled via:

* YAML files


Example:

```yaml
update_rates_hz:
  position:  5
  velocity:  20
  acceleration: 100
  attitude:  250
  rate:      1000
  telemetry: 10
```

---

## Architecture

Typical flow:

1. Sensor input / simulated data (with noise modeling)
2. State estimation (EKF)
3. Control (PID / cascaded loops)
4. Allocation (mapping control to actuators)
5. Output / logging

---

## Roadmap

* [ ] Integrate EKF into flight stack
* [ ] Improve Schedular(transition from std::function to ID dispatch for deterministic behavior)
* [ ] Test waypoint flight
* [ ] Unreal Engine visualization
* [ ] Add hardware abstraction layer


## License

> [MIT]()

---

## Contact

> Anthony Mendoza / anthonymend515@gmail.com / [Linkedin](https://www.linkedin.com/in/anthony-mendoza-3934b0192/)
