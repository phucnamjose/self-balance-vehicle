# Self-Balancing Vehicle

A two-wheeled self-balancing robot on an **ESP32** (ESP-IDF, FreeRTOS), with an
MPU6050 IMU, encoders, an XY-160D motor driver, and a Wi-Fi dashboard.

We are building it **incrementally**, learning each piece in turn, as one
evolving firmware project. Progress and the full task list live in
[docs/ROADMAP.md](docs/ROADMAP.md).

## Repository layout

```
self-balance-vehicle/
├── firmware/      # the ESP-IDF project (currently: Hello world)
├── simulation/    # MATLAB model + tuning (Phase 8 reference)
├── docs/          # ROADMAP, setup guide, hardware & control reference
└── README.md
```
