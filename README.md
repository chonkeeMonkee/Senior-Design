
For launching UI: 
```
python3 launch.py
```

UI will begin once the ESP32 connection is established  

**Code Contributions**
Emmanuel Che - Implemented core IMU drivers to extract motion and orientation data

Armando Zepeda - Implemented Motor Control firmware with safety and fault checks for overcurrent protection, preliminary control operator design, and initial CAN/UI integration for motor actuation.

Alexander Montano-Leon: Implemented firmware for the Power Distribution board, including servo actuation and power monitoring.  Designed and implemented a full multi board CAN communication protocol for telemetry and system control. Integrated CAN protocol into the thruster control firmware to enable command handling and telemetry data transmission. Full system integration for the control operator and real-time telemetry updates via WebSocket communication protocol. Implemented a PID control system for pitch and yaw correction, ensuring continued straight line motion.

Repostory Managed by Alexander Montano-Leon (chonkeeMonkee)
