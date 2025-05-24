# Rocket Simulation
## ✅ Features
- [x] All control calls should retrieve values from StateEstimatio*
- [x] Add **inertia** model to rocket engine to avoid PWM-like control behavior
- [ ] massive restructuring for static library in unreal
## 🐛 Bugs
- [ ] Vehicle **landing** no longer works; likely caused by noise in **StateEstimation** from GPS jitter (random position spikes)

## 🚀 Optimizations
- [ ] Pick a **COTS** sensor and implement its behavior from datasheet into code
- [x] Create or update a **TOML config** per sensor with details from its datasheet
