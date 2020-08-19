# IMU-GPS_Fusion

##### A collection of arduino examples to run on the Sparkfun Razor 9dof IMU M0 (SAMD21 based boards)

### To-Do
- Adjust complimentary filter gain
- Function to remove gravity acceleration vector (output dynamic accerleration only)
- Implement Haversine Formula (or small displacement alternative) to convert lat/lng to displacement (meters)
- Implement Kalman Filter (or EKF) for sensor fusion

### Usage
- Install libraries in arduino ide
- Connect hardware (GPS) `Tx Rx Vcc Gnd`
- Flash board with desired `.ino`

### Resources
- http://arduiniana.org/libraries/tinygpsplus/
- https://github.com/PowerBroker2/NEO-6M_GPS
- https://github.com/rfetick/Kalman
- https://github.com/kriswiner/MPU9250
