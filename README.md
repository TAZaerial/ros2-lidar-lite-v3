# Garmin LiDAR-Lite v3 ROS2 Node Driver

## Installation
- Enable the I2C
```
sudo apt-get install raspi-config
sudo raspi-config
-> Interface Options
--> I2C
---> Enable
```
- Install dependencies
```
sudo apt-get install i2c-tools libi2c-dev python3-smbus
```
- Test using `i2cdetect`
```
sudo i2cdetect-y 1 (or 0)
```
- If address matrix/array appear, then i2c can be used successfully
- If you connect other devices to the i2c, number will be appear filling the "--" array with number, for example 62 (meaning 0x62)

# Source
https://github.com/garmin/LIDARLite_RaspberryPi_Library