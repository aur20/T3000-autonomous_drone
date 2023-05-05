# Arduino reading ultrasonic sensor and filtering data

Small side project. Reads 4 sensors, makes Arduino an I2C slave, RPI requests data from Arduino.

Part list:

 * Arduino Nano
 * 4 * [Ultrasonic Ranging Module HC - SR04](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)
 * RPI reading data via I2C via Python
 * Jumper cables -> wire up as described in the top lines of Arduino script
 * **No additional resistor required**

## Usage

To compile Arduino from CLI, copy ultrasonic_arduino folder to your home, then:

```
arduino-cli compile -p /dev/ttyUSB0 --fqbn arduino:avr:nano ultrasonic_arduino
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano ultrasonic_arduino
```

Install packages at RPI:

```
pip install smbus2
```

## Run the scripts:

```python ultrasonic_i2c_reader.py```:
 * Press ```t``` to set current ambient temperature (default is 20 degree Celsius). Next you are prompted to enter a new value.
 * Press either ```a``` to get all sensor data or type ```1``` to ```4``` get data from one sensor.
 * Press ```r``` to get all raw unfiltered sensor data.

Or use ```ultrasonic_i2c_csvwriter.py``` to constantly monitor data and create a table (which can be fed i.e. to Matlab).

## Sources

https://medium.com/geekculture/raspberry-pi-python-libraries-for-i2c-spi-uart-3df092aeda42
https://pypi.org/project/smbus2/
https://stackoverflow.com/questions/21073085/how-to-send-4-pot-values-via-i2c-from-arduino-to-arduino-how-to-differentiate-t
https://create.arduino.cc/projecthub/B45i/getting-started-with-arduino-cli-7652a5
