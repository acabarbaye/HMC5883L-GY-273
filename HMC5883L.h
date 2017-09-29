/*
 * @file HMC5883L.h
 *
 * @section LICENSE
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 * Module GY-273 - HMC5883L 3-Axis Digital Compass IC
 * Based on library from Tyler Weaver and Oskar Lopez de Gamboa.
 *
 */

#ifndef COMPASS_HMC5883L_H
#define COMPASS_HMC5883L_H

#include "logging.h"
#include "types.h"
#include "mbed.h"

/*
 * Defines
 */

//-----------
// I2C
//-----------
//As stated in datasheet (p9), the compass supports several I2C speeds
#define COMPASS_I2C_STANDARD_MODE    100000
#define COMPASS_I2C_FAST_MODE    400000

//-----------
// Registers
//-----------
#define COMPASS_CONFIG_A_REG    0x00
#define COMPASS_CONFIG_B_REG    0x01
#define COMPASS_MODE_REG        0x02
#define COMPASS_OUTPUT_REG      0x03
#define COMPASS_STATUS_REG      0x09

//-----------
// Configuration register a
//-----------
#define COMPASS_AVG1_SAMPLES    0x00
#define COMPASS_AVG2_SAMPLES    0x20
#define COMPASS_AVG4_SAMPLES    0x40
#define COMPASS_AVG8_SAMPLES    0x60

//-----------
// Data output rates
//-----------
#define COMPASS_OUTPUT_RATE_0_75    0x00
#define COMPASS_OUTPUT_RATE_1_5     0x04
#define COMPASS_OUTPUT_RATE_3       0x08
#define COMPASS_OUTPUT_RATE_7_5     0x0C
#define COMPASS_OUTPUT_RATE_15      0x10
#define COMPASS_OUTPUT_RATE_30      0x14
#define COMPASS_OUTPUT_RATE_75      0x18

//-----------
// Measurement modes
//-----------
#define COMPASS_NORMAL_MEASUREMENT  0x00
#define COMPASS_POSITIVE_BIAS       0x01
#define COMPASS_NEGATIVE_BIAS       0x02

//-----------
// Mode of operation
//-----------
#define COMPASS_CONTINUOUS_MODE                 0x00
#define COMPASS_SINGLE_MEASUREMENT_MODE         0x01
#define COMPASS_IDLE_MODE                       0x02

//-----------
// Gain settings
//-----------
#define COMPASS_GAIN_1370 static_cast<int8_t>(0x00)
#define COMPASS_GAIN_1090 static_cast<int8_t>(0x20)
#define COMPASS_GAIN_820  static_cast<int8_t>(0x40)
#define COMPASS_GAIN_660  static_cast<int8_t>(0x60)
#define COMPASS_GAIN_440  static_cast<int8_t>(0x80)
#define COMPASS_GAIN_390  static_cast<int8_t>(0xA0)
#define COMPASS_GAIN_330  static_cast<int8_t>(0xC0)
#define COMPASS_GAIN_230  static_cast<int8_t>(0xE0)

//-----------
// Status register
//-----------
#define COMPASS_STATUS_LOCK         0x02
#define COMPASS_STATUS_READY        0x01

//-----------
// Measurement error value
//-----------
#define COMPASS_ERROR_VALUE -4096

//-----------
// States
//-----------
typedef enum {
	COMPASS_START,
	COMPASS_MEASURING,
	COMPASS_IDLE,
	COMPASS_SELF_TESTING,
	COMPASS_FAILURE
} eCompassState_t;

/**
 * The HMC5883L 3-Axis Digital Compass IC
 */
class HMC5883L {

public:

	/**
	 * As stated in datasheet (P11), the I2C address of the compass is of 7 bits and is 0x1E. The mbed API uses 8 bit addresses,
	 *  so we need to make sure to left-shift 7 bit address by 1 bit before passing the address.
	 *  0x1E << 1 = 0x3C
	 */
	static const int I2C_ADDRESS = 0x3C;

	/**
	 * Constructor that accepts external i2c interface and interrupt input objects
	 *
	 * Calls init function
	 *
	 * @param i2c The I2C interface object to use.
	 * @param drdy The digital input object to use
	 */
	HMC5883L(I2C &i2c, InterruptIn &drdy);
	/**
	 * Destructor
	 */
	~HMC5883L();
	/**
	 * Function for retrieving the contents of configuration register A
	 *
	 * @returns Configuration Register A
	 */
	int8_t getConfigurationA();
	/**
	 * Function for retrieving the contents of configuration register B
	 *
	 * @returns Configuration Register B
	 */
	int8_t getConfigurationB();
	/**
	 * Function for retrieving the contents of mode register
	 *
	 * @returns mode register
	 */
	int8_t getMode();
	/**
	 * Function for retrieving the raw data
	 * Caution!!  the HMC5883L gives the data in XZY order
	 *
	 * @param output buffer that is at least 3 in length
	 */
	rtStatus_t getRawXZY(int16_t raw[3]);

	/**
	 * Function for getting radian heading using 2-dimensional calculation.
	 *
	 * Note: Compass must be held flat and away from any magnetic field generating
	 * devices such as cell phones and speakers.
	 *
	 * @returns XY heading in radians
	 */
	double getHeadingXY();
	/**
	 * Function for getting radian heading using 2-dimensional calculation.
	 *
	 * @returns YZ heading in radians
	 */
	double getHeadingYZ();
	/**
	 * Function for getting radian heading using 2-dimensional calculation.
	 *
	 * @returns ZX heading in radians
	 */
	double getHeadingZX();

	/**
	 * Function for getting degree heading using 2-dimensional calculation.
	 *
	 * Note: Compass must be held flat and away from an magnetic field generating
	 * devices such as cell phones and speakers.
	 *
	 * @returns XY heading in degrees
	 */
	double getHeadingXYDeg() {
		return CONVERT_RAD_TO_DEG(getHeadingXY());
	}

	/**
	 * Function for getting degree heading using 2-dimensional calculation.
	 *
	 * @returns YZ heading in degrees
	 */
	double getHeadingYZDeg() {
		return CONVERT_RAD_TO_DEG(getHeadingYZ());
	}

	/**
	 * Function for getting degree heading using 2-dimensional calculation.
	 *
	 * @returns ZX heading in degrees
	 */
	double getHeadingZXDeg() {
		return CONVERT_RAD_TO_DEG(getHeadingZX());
	}

	int8_t getDataOutputRate() const {
		return m_data_output_rate;
	}

	void setDataOutputRate(int8_t dataOutputRate) {
		m_data_output_rate = dataOutputRate;
	}

	int8_t getGain() const {
		return m_gain;
	}

	void setGain(int8_t gain) {
		m_gain = gain;
	}

	int_t getI2cSpeed() const {
		return m_i2c_speed;
	}

	void setI2cSpeed(int_t i2cSpeed) {
		m_i2c_speed = i2cSpeed;
	}

	int8_t getAveragingMode() const {
		return m_averaging_mode;
	}

	void setAveragingMode(int8_t averagingMode) {
		m_averaging_mode = averagingMode;
	}
	/**
	 * Function for setting up the compass.
	 */
	rtStatus_t setUp();

	/**
	 * States whether the compass is working properly or not;
	 */
	bool isWorkingProperly() {
		return m_current_state != COMPASS_FAILURE;
	}
	/**
	 * States whether the compass is stopped;
	 */
	bool isStopped() {
		return m_current_state == COMPASS_IDLE;
	}
	/**
	 * States whether the compass is running;
	 */
	bool isRunning() {
		return m_current_state == COMPASS_MEASURING
				|| m_current_state == COMPASS_SELF_TESTING;
	}
	/**
	 * Resets the compass to default configuration.
	 */
	rtStatus_t reset();
	/**
	 * Verifies that compass is healthy by performing a self test.
	 */
	rtStatus_t verify();
	/**
	 * Makes the compass run i.e. measurements are performed and can be retrieved using getRawXZY() or getHeading... methods
	 */
	rtStatus_t run();
	/**
	 * Stops the compass, i.e. puts it into idle mode.
	 */
	rtStatus_t stop();

private:
	Semaphore m_semaphore;
	Mutex m_mutex;
	Thread m_thread;
	I2C &p_i2c;
	InterruptIn &p_drdy;
	bool volatile m_stop_measurements;
	bool volatile m_measurements_have_started;
	char m_measurement_data[6];
	eCompassState_t m_current_state;
	int_t m_i2c_speed;
	int8_t m_gain;
	int8_t m_data_output_rate;
	int8_t m_averaging_mode;

	rtStatus_t configureForMeasurements();
	rtStatus_t configureForSelfTest();
	static void release(void const *argument);
	static void measure(void const *argument);
	static void test(void const *argument);
	void initialise();
	rtStatus_t performMeasurement();
	rtStatus_t runMeasurements();
	rtStatus_t startMeasurements();
	rtStatus_t startSelfTest();
	rtStatus_t stopMeasurements();
	rtStatus_t executeStateMachine(eCompassState_t nextState);
	rtStatus_t runSelfTest();
	rtStatus_t performFullSelfTest();
	rtStatus_t runSingleMeasurement();
	rtStatus_t checkRawOutput(int16_t output[3]);
	rtStatus_t checkLimits(int16_t output[3], int16_t limit_low,
			int16_t limit_high);
	rtStatus_t checkLimits(int16_t output[3]);
	rtStatus_t setConfigurationRegister(int8_t register_address, int8_t value);
	int8_t getConfigurationRegister(int8_t register_address);
	/**
	 * Function for setting configuration register A
	 *
	 * Default value is 8 average Sample per output, 15Hz Data output rate, normal measurement mode
	 *
	 * @param config the value to place in Configuration Register A
	 */
	rtStatus_t setConfigurationA(int8_t value);

	/**
	 * Sets the frequency of I2C
	 * @param frequency COMPASS_I2C_STANDARD_MODE or COMPASS_I2C_FAST_MODE
	 */
	rtStatus_t setI2CSpeedMode(int_t frequency);

	/**
	 * Function for setting configuration register B
	 *
	 * Configuration Register B is for setting the device gain.
	 *
	 * @param config the value to place in Configuration Register B
	 */
	rtStatus_t setConfigurationB(int8_t value);

	/**
	 * Function for setting the mode register
	 *
	 * @param mode the value for setting in the Mode Register
	 */
	rtStatus_t setMode(int8_t mode);

	/**
	 * Function for retrieving the contents of status register
	 *
	 * Bit1: LOCK, Bit0: RDY
	 *
	 * @returns status register
	 */
	uint8_t getStatus();
	/**
	 * Function to get a generic heading. Specify index1 and index2 to define which plane to consider e.g. for XY index1=0 index2=2 and for XZ index1=0 index2=1
	 */
	double getHeading(int index1, int index2);

};

#endif // HMC5883L
