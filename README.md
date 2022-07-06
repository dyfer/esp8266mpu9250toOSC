# esp8266mpu9250toOSC

This repository contains code for sending data from an accelerometer sensor with a microcontroller to another device using OSC.

The microcontroller code is using the Arduino environment (see https://github.com/esp8266/Arduino). The receiving examples located in `sc/` are written in SuperCollider.

## Hardware

- Espressif ESP8266 microcontroller (e.g. [Adafruit HUZZAH](https://learn.adafruit.com/adafruit-huzzah-esp8266-breakout))
- InvenSense MPU-9250 (e.g. GY-9250 board)
- LiPo battery
- power switch
- (optionally) a pair of resistors to make a voltage divider for measuring battery voltage (maximum voltage of ESP8266's ADC is 1V) 

MPU-9250 should be connected to the board using an I2C interface. Most pins can be used for this purpose (refer to ESP8266's documentation). Additionally, there should be one more connection to 9250's interrupt pin (this can be any pin except 16 and probably 0).

MPU-9250 can be powered directly from the battery, or from the HUZZAH's 3.3V regulator.

For programming and setting the microcontroller, one needs to use a serial interface (e.g. USB-serial)

### Voltage measurement
In order to measure voltage with the HUZZAH board, it's necessary to create a voltage divider with two resistors, connecting VBat+ to the ADC input, e.g.
```
VBat+     GND
  \       /
1000kΩ  100kΩ
    \   /
     ADC
```
The values can be different, the point is that they need to bring the battery voltage (e.g. LiPo Battery might have around 4.2V when full) under the 1V max. 

If the value of the resistors is different than here, it should be modified in the code (see below).


## Firmware

### Pin configuration

Pins for the I2C are set when initializing the `Wire` library, where you can adapt them as needed.

```cpp
// initialize accelerometer device
Serial.println("Initializing I2C devices");
Wire.setClock(400000);//speed up I2C
Wire.begin(4, 2); //define which pins are used for I2C: Wire.begin(int sda, int scl)
```

Interrupt pin is set as a variable:
```cpp
// Pin definitions
int intPin = 5;
```

### Battery voltage
If necessary, resistor values for the voltage divider should be updated in the code:
```cpp
//voltage measurement
float r1 = 100; //lower resistor value (kOhm)
float r2 = 1000; //higher resistor value (kOhm)
```

### Flashing

Programing the microcontroller should be straightforward:
- Download board definition and required library (OSC) in Arduino
- Open the `esp8266mpu9250kwiner.ino` file
- Select the appropriate board and serial port
- Compile and upload
    - note that Adafruit HUZZAH needs to be put manually into the firmward upload mode by holding GPIO0 button down while powering on/resetting the board


## Setup

After flashing the firmware, some settings need to be set in order for the system to be operational. Interaction with the microcontroller is facilitated through a serial connection at the baud rate of __115200__ using ASCII characters. 

Parameters that need to be set before using the microcontroller
- sensor name
- WiFi SSID
- WiFi password (if used)
- accelerometer and gyroscope calibration
- magnetometer calibration
- destination IP and port (this can also be set by sending an OSC message to the microcontroller, see SuperCollider example below)

### Commands

In order to read the current value of a parameter, send `<parameterName> <enter>`. To set the parameter, send `<parameterName> <values> <enter>`.

This is the list of available commands:

|command|values|description|
|-|-|-|
|`?`| | print available commands (short) |
|`??`| | print available commands with descritpions |
|`name` | `<string>` | name of the sensor (will also set the OSC path)|
|`ssid` | `<string>` | WiFi network name |
| `pwd`|  `<string>` | WiFi network password (if used); set to `""` to disable |
|`calibrateAccGyro`| | start calibration process for Accelerometer and Gyro; also see below (*)|
| `calibrateMag` | |start calibration process for Accelerometer and Gyro; also see below (*)|
|`ip` | e.g. `192.168.1.10` | destination IP |
|`port` | e.g. `10000` | destination port |
|`myIP`| | get microcontroller's IP |
|`myPort`| | get microcontroller's receiving port |
|`format`| space-delimited list of strings | message format; see formatInfo |
| `formatInfo` | | list available format names |
| `version` | | print version string |
|`oscPath`| | print osc path |
|`fallback`|`0` or `1`| send sensor data over the serial port |
| `status` |  | get information on the current status |

(*) NOTE: currently the microcontroller needs to be connected to the WiFi network in order for the callibration to work

### Calibration

Make sure to be connected to the WiFi network first.

#### Accelerometer and Gyro calibration

Type `calibrateAccGyro`.

The accelerometer should be stable and perfectly flat. 

#### Magnetometer calibration

Type `calibrateMag`.

The sensor should be rotated along all axis for about 15 seconds. Make sure to not be too close to strong magnetic fields.

### Setting send format

OSC messages are sent with the path `/acc/<accelerometerName>`, followed by the defined sensor data.

Available data:

|data|description|
|-|-|
| `acc` | accelerometer data (x, y, z), in `G` (gravity force) |
|`gyro`| gyroscope data (x, y, z) |
| `accgyro` | shorthand for accelerometer and gyroscope data (x, y, z, x, y, z) |
|`yawpitchroll`| yaw, pitch, roll |
|`quaternion`| quaternion data (w, x, y, z) |
|`counter`| counter from 0-255, increments with each message and wraps around, e.g. for detecting lost packets|
|`b1`, `b2`, `b3`,`b4`| button 1-4 data (value 0 or 1, send continuously) |
|`realaccel`| linear acceleration (untested)|
<!-- |`euler` | euler angles (noop)| -->
<!-- |`worldaccel` | (noop)| -->

To set the format, type `format <name1> <name2> <name3> etc`, e.g. `format acc` or `format acc yawpitchroll`.

## Receiving data in SuperCollider

See `sc/` folder for SuperCollider code.

[sensor management.scd](<sc/sensor management.scd>) allows for setting all the devices on the network to send messages to SuperCollider.

[orientation visualization.scd](<sc/orientation visualization.scd>) provides a simple visualization for the `yawpitchroll` data (use `format acc yawpitchroll`).

[accelerometer and granulator.scd](<sc/accelerometer and granulator.scd>) is a simple live sound example that uses accelerometer data.


