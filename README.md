# HMC5883L-GY-273
mbed OS 5 library for 3 Axis Compass HMC5883L Module GY-273

## Description
![](https://i2.wp.com/henrysbench.capnfatz.com/wp-content/uploads/2015/09/GY-273-Magnetometer-Pin-Outs.png)

The GY-273 module is based on the Honeywell HMC5883L ([datasheet](https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf))IC for low-field magnetic sensing with a digital interface for applications such as lowcost compassing and magnetometry. The Mbed board communicates with the sensor using I2C protocol and also uses a digital input (drdy) to know when the sensor is ready

_[For more information about the module.](https://wiki.eprolabs.com/index.php?title=GY-273_Compass_Module)_
## Usage

* Connect the module to the board
```
    VCC -> VCC (3.3V)

    GND -> GND

    SCL -> SCL e.g. D14

    SDA -> SDA e.g. D15

    DRDY -> Digital input e.g. D2
```
* Load a program using the library.

Hereafter is an example program that uses the library:
```cpp
#include "mbed.h"
#include "mbed_config.h"
#include "logging.h"
#include "HMC5883L.h"

DigitalOut led1(LED1);
I2C i2c(I2C_SDA, I2C_SCL);
InterruptIn drdy(D2);
HMC5883L compass(i2c, drdy);

// main() runs in its own thread in the OS
int main() {
	LOG_DEBUG("HELLO - I am a compass")
	compass.setUp();
	LOG_DEBUG_HEXADECIMAL_VALUE("REG A", compass.getConfigurationA())
	LOG_DEBUG_HEXADECIMAL_VALUE("REG B", compass.getConfigurationB())
	LOG_DEBUG_HEXADECIMAL_VALUE("mode", compass.getMode())
	compass.run();
	int16_t data[3];
	LOG_DEBUG("ABOUT TO START")
	while (true) {
		led1 = !led1;
		if (compass.getRawXZY(data) == SUCCEEDED) {
			LOG_DEBUG("Compass values - SUCCESS")
			LOG_DEBUG_INTEGER_VALUE("X", data[0])
			LOG_DEBUG_INTEGER_VALUE("Y", data[2])
			LOG_DEBUG_INTEGER_VALUE("Z", data[1])
			LOG_DEBUG_DOUBLE_VALUE("XY", compass.getHeadingXYDeg())
			LOG_DEBUG_DOUBLE_VALUE("YZ", compass.getHeadingYZDeg())
			LOG_DEBUG_DOUBLE_VALUE("ZX", compass.getHeadingZXDeg())

		} else {
			LOG_DEBUG("Compass values - FAILURE")
		}
		wait(2);
	}
}
```