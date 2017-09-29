/*
 * @file HMC5883L.cpp
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
 * HMC5883L 3-Axis Digital Compas IC
 * Based on library from Tyler Weaver and Oskar Lopez de Gamboa.
 *
 */

#include "HMC5883L.h"

HMC5883L::HMC5883L(I2C &i2c, InterruptIn &drdy) :
		p_i2c(i2c), p_drdy(drdy) {
	initialise();
}

HMC5883L::~HMC5883L() {

}

rtStatus_t HMC5883L::setI2CSpeedMode(int_t frequency) {
	p_i2c.frequency(frequency);
	return SUCCEEDED;
}

rtStatus_t HMC5883L::setConfigurationRegister(int8_t register_address,
		int8_t config) {
	char config_value = static_cast<char>(config);
	char address = static_cast<char>(register_address);
	char cmd[2];
	cmd[0] = address; // register a address
	cmd[1] = config_value;
	CHECK_SUCCESS(p_i2c.write(I2C_ADDRESS, cmd, 2), I2C_ACKNOWLEDGEMENT)
	return SUCCEEDED;
}

rtStatus_t HMC5883L::setConfigurationA(int8_t config) {
	LOG_TRACE("HMC5883L::setConfigurationA()")
	LOG_TRACE_HEXADECIMAL_VALUE("registerA", static_cast<char>(config))
	return setConfigurationRegister(COMPASS_CONFIG_A_REG, config);

}

rtStatus_t HMC5883L::setConfigurationB(int8_t config) {
	LOG_TRACE("HMC5883L::setConfigurationB()")
	LOG_TRACE_HEXADECIMAL_VALUE("registerB", static_cast<char>(config))
	return setConfigurationRegister(COMPASS_CONFIG_B_REG, config);
}

rtStatus_t HMC5883L::setMode(int8_t mode) {
	LOG_TRACE("HMC5883L::setMode()")
	LOG_TRACE_HEXADECIMAL_VALUE("mode", static_cast<char>(mode))
	return setConfigurationRegister(COMPASS_MODE_REG, mode);
}

int8_t HMC5883L::getConfigurationRegister(int8_t register_address) {
	if (m_measurements_have_started) {
		LOG_DEBUG(
				"Error: Measurements are currently running. Please stop them to be able to carry out this action")
		return COMPASS_ERROR_VALUE;
	}
	char cmd[1], result[1];
	cmd[0] = static_cast<char>(register_address); // register a address
	p_i2c.write(I2C_ADDRESS, cmd, 1, true);
	p_i2c.read(I2C_ADDRESS, &result[0], 1, false);
	return result[0];
}

int8_t HMC5883L::getConfigurationA() {
	return getConfigurationRegister(COMPASS_CONFIG_A_REG);
}

int8_t HMC5883L::getConfigurationB() {
	return getConfigurationRegister(COMPASS_CONFIG_B_REG);
}

int8_t HMC5883L::getMode() {
	return getConfigurationRegister(COMPASS_MODE_REG);
}

uint8_t HMC5883L::getStatus() {
	return getConfigurationRegister(COMPASS_STATUS_REG);
}

void HMC5883L::initialise() {
	setI2cSpeed(COMPASS_I2C_FAST_MODE);
	setAveragingMode(COMPASS_AVG8_SAMPLES);
	setDataOutputRate(COMPASS_OUTPUT_RATE_15);
	setGain(COMPASS_GAIN_1090);
	m_stop_measurements = true;
	m_measurements_have_started = false;
	m_current_state = COMPASS_START;
}

rtStatus_t HMC5883L::setUp() {
	LOG_DEBUG("HMC5883L::setUp()")
	p_drdy.rise(callback(release, this));
	return executeStateMachine(COMPASS_START);
}

rtStatus_t HMC5883L::configureForMeasurements() {
	LOG_TRACE("HMC5883L::configureForMeasurements()")
	CHECK_SUCCESS(setI2CSpeedMode(m_i2c_speed), SUCCEEDED)
	// Set register A
	CHECK_SUCCESS(
			setConfigurationA( m_averaging_mode | m_data_output_rate | COMPASS_NORMAL_MEASUREMENT),
			SUCCEEDED)
	// Set gain
	CHECK_SUCCESS(setConfigurationB(m_gain), SUCCEEDED)
	// Set measurement mode
	CHECK_SUCCESS(setMode(COMPASS_CONTINUOUS_MODE), SUCCEEDED)
	return SUCCEEDED;
}

rtStatus_t HMC5883L::configureForSelfTest() {
	LOG_TRACE("HMC5883L::configureForSelfTest()")
	CHECK_SUCCESS(setI2CSpeedMode(m_i2c_speed), SUCCEEDED)
	// Set register A
	CHECK_SUCCESS(
			setConfigurationA( m_averaging_mode | m_data_output_rate | COMPASS_POSITIVE_BIAS),
			SUCCEEDED)
	// Set gain
	CHECK_SUCCESS(setConfigurationB(m_gain), SUCCEEDED)
	// Set measurement mode
	CHECK_SUCCESS(setMode(COMPASS_CONTINUOUS_MODE), SUCCEEDED)
	return SUCCEEDED;
}

rtStatus_t HMC5883L::executeStateMachine(eCompassState_t nextState) {
	LOG_TRACE("HMC5883L::executeStateMachine()")
	rtStatus_t result = SUCCEEDED;
	if (m_current_state == COMPASS_FAILURE) {
		if (nextState != COMPASS_START) {
			nextState = COMPASS_FAILURE;
		}
	}
	switch (nextState) {
	case COMPASS_START:
		initialise();
		break;
	case COMPASS_MEASURING:
		if (m_current_state != COMPASS_MEASURING) {
			if (m_current_state == COMPASS_SELF_TESTING) {
				result = stopMeasurements();
			}
			if (result == SUCCEEDED) {
				result = startMeasurements();
			}
		}
		break;
	case COMPASS_IDLE:
		if (m_current_state != COMPASS_IDLE) {
			result = stopMeasurements();
		}
		break;
	case COMPASS_SELF_TESTING:
		if (m_current_state != COMPASS_SELF_TESTING) {
			if (m_current_state == COMPASS_MEASURING) {
				result = stopMeasurements();
			}
			if (result == SUCCEEDED) {
				result = performFullSelfTest();
			}
		}
		break;
	case COMPASS_FAILURE:
		if (m_stop_measurements == false) {
			stopMeasurements();
		}
		result = FAILED;
		break;
	}
	if (result == SUCCEEDED) {
		m_current_state = nextState;
	} else {
		m_current_state = COMPASS_FAILURE;
	}
	return result;
}

rtStatus_t HMC5883L::startMeasurements() {
	LOG_TRACE("HMC5883L::startMeasurements()")
	m_stop_measurements = false;
	m_measurements_have_started = false;
	CHECK_SUCCESS(m_thread.start(callback(measure, this)), osOK)
	return SUCCEEDED;
}
rtStatus_t HMC5883L::startSelfTest() {
	LOG_TRACE("HMC5883L::startSelfTest()")
	m_stop_measurements = false;
	m_measurements_have_started = false;
	CHECK_SUCCESS(m_thread.start(callback(test, this)), osOK)
	return SUCCEEDED;
}

rtStatus_t HMC5883L::performFullSelfTest() {
	LOG_TRACE("HMC5883L::performFullSelfTest()")
	CHECK_SUCCESS(startSelfTest(), SUCCESS)
	if (m_measurements_have_started == false) {
		Thread::wait(50);
	}
	CHECK_SUCCESS(m_measurements_have_started, true)
	int16_t output[3];
	CHECK_SUCCESS(getRawXZY(output), SUCCESS)
	return checkLimits(output);
}

rtStatus_t HMC5883L::stopMeasurements() {
	LOG_TRACE("HMC5883L::stopMeasurements()")
	m_measurements_have_started = false;
	if (m_stop_measurements == false) {
		m_stop_measurements = true;
		CHECK_SUCCESS(m_thread.join(), osOK)
	}
	return SUCCEEDED;
}

void HMC5883L::measure(void const *argument) {
	HMC5883L* self = (HMC5883L*) argument;
	self->runMeasurements();
}
void HMC5883L::test(void const *argument) {
	HMC5883L* self = (HMC5883L*) argument;
	self->runSelfTest();
}

void HMC5883L::release(void const *argument) {
	HMC5883L* self = (HMC5883L*) argument;
	self->performMeasurement();
}

rtStatus_t HMC5883L::getRawXZY(int16_t output[3]) {
	LOG_DEBUG("HMC5883L::getRawXZY()")
	if (m_measurements_have_started == false) {
		return FAILED;
	}
	CHECK_SUCCESS(m_mutex.lock(), osOK)
	for (int i = 0; i < 3; i++) { // fill the output variables
		output[i] =
				static_cast<int16_t>(static_cast<int16_t>(m_measurement_data[i
						* 2]) << 8
						| static_cast<int16_t>(m_measurement_data[i * 2 + 1]));
	}
	CHECK_SUCCESS(m_mutex.unlock(), osOK)
	return checkRawOutput(output);
}

rtStatus_t HMC5883L::checkRawOutput(int16_t output[3]) {
	LOG_TRACE("HMC5883L::checkRawOutput()")
	//In datasheet (p15), it is stated that in the event of reading underflow or overflow, the data register will contain the value COMPASS_ERROR_VALUE
	for (int i = 0; i < 3; i++) {
		if (output[i] == COMPASS_ERROR_VALUE) {
			return FAILED;
		}
	}
	return SUCCEEDED;
}

rtStatus_t HMC5883L::checkLimits(int16_t output[3], int16_t limit_low,
		int16_t limit_high) {
	for (int i = 0; i < 3; i++) {
		int16_t value = output[i];
		if (value < limit_low || value > limit_high) {
			return FAILED;
		}
	}
	return SUCCEEDED;
}

rtStatus_t HMC5883L::checkLimits(int16_t output[3]) {
	LOG_TRACE("HMC5883L::checkLimits()")
	rtStatus_t result = SUCCEEDED;
	switch (m_gain) {
	case COMPASS_GAIN_390: // Gain 5
	result = checkLimits(output, 243, 575);
	break;
	case COMPASS_GAIN_330:// Gain 6
	result = checkLimits(output, 206, 487);
	break;
	case COMPASS_GAIN_230:// Gain 7
	result = checkLimits(output, 143, 339);
	break;
}
	return result;
}

rtStatus_t HMC5883L::runMeasurements() {
	LOG_TRACE("HMC5883L::runMeasurements()")
	CHECK_SUCCESS(configureForMeasurements(), SUCCEEDED)
	while (!m_stop_measurements) {
		m_semaphore.wait();
		CHECK_SUCCESS(runSingleMeasurement(), SUCCEEDED)
	}
	return SUCCEEDED;
}

rtStatus_t HMC5883L::runSelfTest() {
	LOG_TRACE("HMC5883L::runSelfTest()")
	CHECK_SUCCESS(configureForSelfTest(), SUCCEEDED)
	while (!m_stop_measurements) {
		m_semaphore.wait();
		CHECK_SUCCESS(runSingleMeasurement(), SUCCEEDED)
	}
	return SUCCEEDED;
}

rtStatus_t HMC5883L::performMeasurement() {
	m_semaphore.release();
	return SUCCEEDED;
}

rtStatus_t HMC5883L::runSingleMeasurement() {
//	LOG_TRACE("HMC5883L::runSingleMeasurement()")
	char cmd[1] = { 0x03 };
	CHECK_SUCCESS(m_mutex.lock(), osOK)
	// sets the pointer to the start of x
	CHECK_SUCCESS(p_i2c.write(I2C_ADDRESS, cmd, 1, true), I2C_ACKNOWLEDGEMENT)
	CHECK_SUCCESS(p_i2c.read(I2C_ADDRESS, m_measurement_data, 6, false),
			I2C_ACKNOWLEDGEMENT)
	CHECK_SUCCESS(m_mutex.unlock(), osOK)
	m_measurements_have_started = true;
	return SUCCEEDED;
}

double HMC5883L::getHeading(int index1, int index2) {
	LOG_DEBUG("HMC5883L::getHeading()")
	int16_t raw_data[3];
	if (getRawXZY(raw_data) == FAILED) {
		return 0.0f;
	}
	//The  HMC5883L gives X Z Y order
	double heading = atan2(static_cast<double>(raw_data[index2]),
			static_cast<double>(raw_data[index1])); // e.g. heading = arctan(Y/X) for XY
	if (heading < 0.0f) { // fix sign
		heading += M_2PI;
	}

	if (heading > M_2PI) { // fix overflow
		heading -= M_2PI;
	}

	return heading;
}
double HMC5883L::getHeadingXY() {
	return getHeading(0, 2);
}

double HMC5883L::getHeadingYZ() {
	return getHeading(2, 1);
}

double HMC5883L::getHeadingZX() {
	return getHeading(1, 0);
}

rtStatus_t HMC5883L::reset() {
	LOG_DEBUG("HMC5883L::reset()")
	return executeStateMachine(COMPASS_START);
}

rtStatus_t HMC5883L::verify() {
	LOG_DEBUG("HMC5883L::verify()")
	return executeStateMachine(COMPASS_SELF_TESTING);
}

rtStatus_t HMC5883L::run() {
	LOG_DEBUG("HMC5883L::run()")
	return executeStateMachine(COMPASS_MEASURING);
}

rtStatus_t HMC5883L::stop() {
	LOG_DEBUG("HMC5883L::stop()")
	return executeStateMachine(COMPASS_IDLE);
}
