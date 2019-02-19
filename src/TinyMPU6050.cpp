/*
 *	Register map for the MPU6050 available at: https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 */
#include "TinyMPU6050.h"
#include "Arduino.h"

/*
 *	Constructor
 */
MPU6050::MPU6050(TwoWire &w) {

	// Setting I²C stuff
	wire = &w;
}

/*
 *	Initialization method
 */
void Initialize () {

	// Setting attributes with default values
	filterAccelCoeff = 0.03f;
	filterGyroCoeff = 0.97f;

	// Setting sample rate divider
	RegisterWrite(MPU6050_SMPLRT_DIV, 0x00);

	// Setting frame synchronization and the digital low-pass filter
	RegisterWrite(MPU6050_CONFIG, 0x00);

	// Setting gyro self-test and full scale range
	RegisterWrite(MPU6050_GYRO_CONFIG, 0x08);

	// Setting accelerometer self-test and full scale range
	RegisterWrite(MPU6050_ACCEL_CONFIG, 0x00);

	// Waking up MPU6050
	RegisterWrite(MPU6050_PWR_MGMT_1, 0x01);

	// Setting angles to zero
	angX = 0;
	angY = 0;
	angZ = 0;

	// Beginning integration interval
	intervalStart = millis();
}

/*
 *	Register write method
 */
void RegisterWrite (byte registerAddress, byte data) {

	// Starting transmission for MPU6050
	wire->beginTransmission(MPU6050_ADDRESS);

	// Accessing register
	wire->write(registerAddress);

	// Writing data
	wire->write(data);

	// Closing transmission
	wire->endTransmission();
}

/*
 *	Gyro calibration method
 */
void CalibrateGyro () {

	// TODO: What to do?
}

/*
 *	Set function for gyro offsets
 */
void SetGyroOffsets (float x, float y, float z) {

	// Setting offsets
	gyroXOffset = x;
	gyroYOffset = y;
	gyroZOffset = z;
}

/*
 *	Set function for the acc coefficient on complementary filter
 */
void SetFilterAccCoeff (float coeff) {

	// Setting coefficient
	filterAccelCoeff = coeff;
}

/*
 *	Set function for the gyro coefficient on complementary filter
 */
void SetFilterGyroCoeff (float coeff) {

	// Setting coefficient
	filterGyroCoeff = coeff;
}