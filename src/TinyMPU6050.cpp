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

	// Beginning integration timer
	intervalStart = millis();
}

/*
 *	Method that executes all readings and updates all attributes
 */
void Execute () {

	// Updating raw data before processing it
	this->UpdateRawAccel();
	this->UpdateRawGyro();

	// TODO:
	// - Subtract accel/gyro offsets;
	// - Readable accel/gyro data;
	// - Accel/Gyro angles;
	// - Complementary filter angles.

	// Reseting the integration timer
	intervalStart = millis();
}

/*
 *	Raw accel data update method
 */
void UpdateRawAccel () {

	// Beginning transmission for MPU6050
	wire->beginTransmission(MPU6050_ADDRESS);

	// Accessing accel data registers
	wire->write(MPU6050_ACCEL_XOUT_H);
	wire->endTransmission(false);

	// Requesting accel data
	wire->requestFrom((int) MPU6050_ADDRESS, 6, (int) true);

	// Storing raw accel data
	rawAccX = wire->read() << 8 | wire->read();
	rawAccY = wire->read() << 8 | wire->read();
	rawAccZ = wire->read() << 8 | wire->read();
}

/*
 *	Raw gyro data update method
 */
void UpdateRawGyro () {

	// Beginning transmission for MPU6050
	wire->beginTransmission(MPU6050_ADDRESS);

	// Accessing gyro data registers
	wire->write(MPU6050_GYRO_XOUT_H);
	wire->endTransmission(false);

	// Requesting gyro data
	wire->requestFrom((int) MPU6050_ADDRESS, 6, (int) true);

	// Storing raw gyro data
	rawGyroX = wire->read() << 8 | wire->read();
	rawGyroY = wire->read() << 8 | wire->read();
	rawGyroZ = wire->read() << 8 | wire->read();
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
 *	MPU-6050 calibration method inspired by https://42bots.com/tutorials/arduino-script-for-mpu-6050-auto-calibration/
 */
void Calibrate () {

	// Reading data (DISCARDED_MEASURES) times without storing
	for (byte i = 0; i < DISCARDED_MEASURES; i++) {
		this->UpdateRawAccel();
		this->UpdateRawGyro();
		// Recommended delay for new reading
		delay(2);
	}

	// Reading data (CALIBRATION_MEASURES) times adding its values to "long" vars
	long sumAccX, sumAccY, sumAccZ, sumGyroX, sumGyroY, sumGyroZ = 0;
	for (int i = 0; i < CALIBRATION_MEASURES; i++) {
		this->UpdateRawAccel();
		this->UpdateRawGyro();
		sumAccX += rawAccX;
		sumAccY += rawAccY;
		sumAccZ += rawAccZ;
		sumGyroX += rawGyroX;
		sumGyroY += rawGyroY;
		sumGyroZ += rawGyroZ;
		// Recommended delay for new reading
		delay(2);
	}

	// Dividing sums by (CALIBRATION_MEASURES) and definig "preOffsets"
	int16_t preOffAccX, preOffAccY, preOffAccZ, preOffGyroX, preOffGyroY, preOffGyroZ = 0;
	preOffAccX = (sumAccX / CALIBRATION_MEASURES) / ACCEL_PREOFFSET_MAGIC_NUMBER;
	preOffAccY = (sumAccY / CALIBRATION_MEASURES) / ACCEL_PREOFFSET_MAGIC_NUMBER;
	preOffAccZ = (16384 - (sumAccZ / CALIBRATION_MEASURES)) / ACCEL_PREOFFSET_MAGIC_NUMBER;
	preOffGyroX = (sumGyroX / CALIBRATION_MEASURES) / GYRO_PREOFFSET_MAGIC_NUMBER;
	preOffGyroY = (sumGyroY / CALIBRATION_MEASURES) / GYRO_PREOFFSET_MAGIC_NUMBER;
	preOffGyroZ = (sumGyroZ / CALIBRATION_MEASURES) / GYRO_PREOFFSET_MAGIC_NUMBER;

	// Checking if readings are on the deadzone. If they are not, fix them
	while (true) {

		// Control variable
		byte ready = 0;

		// Setting sums to zero
		sumAccX, sumAccY, sumAccZ, sumGyroX, sumGyroY, sumGyroZ = 0; 

		// Making few readings, summing values and subtracting preOffsets
		for (byte i = 0; i < CHECKING_MEASURES; i++) {
			this->UpdateRawAccel();
			this->UpdateRawGyro();
			sumAccX += rawAccX - preOffAccX;
			sumAccY += rawAccY - preOffAccY;
			sumAccZ += rawAccZ - preOffAccZ;
			sumGyroX += rawGyroX - preOffGyroX;
			sumGyroY += rawGyroY - preOffGyroY;
			sumGyroZ += rawGyroZ - preOffGyroZ;
			// Recommended delay for new reading
			delay(2);
		}

		// Computing averages
		sumAccX = (sumAccX / CALIBRATION_MEASURES);
		sumAccY = (sumAccY / CALIBRATION_MEASURES);
		sumAccZ = (16384 - (sumAccZ / CALIBRATION_MEASURES));
		sumGyroX = (sumGyroX / CALIBRATION_MEASURES);
		sumGyroY = (sumGyroY / CALIBRATION_MEASURES);
		sumGyroZ = (sumGyroZ / CALIBRATION_MEASURES);

		// Checking if readings are on the deadzone and, eventually, fixing preOffsets
		if (abs(sumAccX) <= ACCEL_DEADZONE) ready++;
		else preOffAccX = preOffAccX - sumAccX / ACCEL_DEADZONE;

		if (abs(sumAccY) <= ACCEL_DEADZONE) ready++;
		else preOffAccY = preOffAccY - sumAccY / ACCEL_DEADZONE;

		if (abs(16384 - sumAccZ) <= ACCEL_DEADZONE) ready++;
		else preOffAccZ = preOffAccZ - sumAccZ / ACCEL_DEADZONE;

		if (abs(sumGyroX) <= GYRO_DEADZONE) ready++;
		else preOffGyroX = preOffGyroX - sumGyroX / (GYRO_DEADZONE + 1);

		if (abs(sumGyroY) <= GYRO_DEADZONE) ready++;
		else preOffGyroY = preOffGyroY - sumGyroY / (GYRO_DEADZONE + 1);

		if (abs(sumGyroZ) <= GYRO_DEADZONE) ready++;
		else preOffGyroZ = preOffGyroZ - sumGyroZ / (GYRO_DEADZONE + 1);

		// Checking if everything's ready
		if (ready == 6) break;
	}

	// Setting the offsets
	this->SetAccOffsets(preOffAccX, preOffAccY, preOffAccZ);
	this->SetGyroOffsets(preOffGyroX, preOffGyroY, preOffGyroZ);
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
 *	Set function for accel offsets
 */
void SetAccOffsets (float x, float y, float z) {

	// Setting offsets
	accXOffset = x;
	accYOffset = y;
	accZOffset = z;
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