# Rocket Simulation
## ✅ Features
- [ ] All control calls should retrieve values from StateEstimatio*
- [ ] Add **inertia** model to rocket engine to avoid PWM-like control behavior

## 🐛 Bugs
- [ ] Vehicle **landing** no longer works; likely caused by noise in **StateEstimation** from GPS jitter (random position spikes)

## 🚀 Optimizations
- [ ] Pick a **COTS** sensor and implement its behavior from datasheet into code
- [ ] Create or update a **TOML config** per sensor with details from its datasheet
