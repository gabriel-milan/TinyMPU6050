/*
 *	Register map for the MPU6050 available at: https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 */
#include "TinyMPU6050.h"

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
void MPU6050::Initialize () {

	// Beginning Wire
	wire->begin();

	// Setting attributes with default values
	filterAccelCoeff = DEFAULT_ACCEL_COEFF;
	filterGyroCoeff = DEFAULT_GYRO_COEFF;

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

	// Setting deadzones
	SetAccelDeadzone(DEFAULT_ACCEL_DEADZONE);
	SetGyroDeadzone(DEFAULT_GYRO_DEADZONE);
}

/*
 *	Method that executes all readings and updates all attributes
 */
void MPU6050::Execute () {

	// Updating raw data before processing it
	this->UpdateRawAccel();
	this->UpdateRawGyro();

	// TODO:
	// - Complementary filter angles.

	// Computing readable accel/gyro data
	accX = (float)(rawAccX - accXOffset) / ACCEL_TRANSFORMATION_NUMBER;
	accY = (float)(rawAccY - accYOffset) / ACCEL_TRANSFORMATION_NUMBER;
	accZ = (float)(rawAccZ - accZOffset) / ACCEL_TRANSFORMATION_NUMBER;
	gyroX = (float)(rawGyroX - gyroXOffset) / GYRO_TRANSFORMATION_NUMBER;
	gyroY = (float)(rawGyroY - gyroYOffset) / GYRO_TRANSFORMATION_NUMBER;
	gyroZ = (float)(rawGyroZ - gyroZOffset) / GYRO_TRANSFORMATION_NUMBER;

	float auxAccZ = (float)(ACCEL_TRANSFORMATION_NUMBER - rawAccZ - accZOffset) / ACCEL_TRANSFORMATION_NUMBER;

	// Computing accel angles
	angAccX = (atan2(-accY, -auxAccZ)) * RAD_TO_DEG;
	angAccY = (atan2(-accX, -auxAccZ)) * RAD_TO_DEG;
	angAccZ = (atan2(-accY, -accX)) * RAD_TO_DEG;

	// Computing gyro angles
	dt = (millis() - intervalStart) * 0.001;
	angGyroX += gyroX * dt;
	angGyroY += gyroY * dt;
	angGyroZ += gyroZ * dt;

	// Computing complementary filter angles
	angX = (filterAccelCoeff * angAccX) + (filterGyroCoeff * (angX + gyroX * dt));
	angY = (filterAccelCoeff * angAccY) + (filterGyroCoeff * (angY + gyroY * dt));
	angZ = (filterAccelCoeff * angAccZ) + (filterGyroCoeff * (angZ + gyroZ * dt));

	// Reseting the integration timer
	intervalStart = millis();
}

/*
 *	Raw accel data update method
 */
void MPU6050::UpdateRawAccel () {

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
void MPU6050::UpdateRawGyro () {

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
void MPU6050::RegisterWrite (byte registerAddress, byte data) {

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
void MPU6050::Calibrate (bool console) {

	// Reading data (DISCARDED_MEASURES) times without storing
	for (byte i = 0; i < DISCARDED_MEASURES; i++) {
		this->UpdateRawAccel();
		this->UpdateRawGyro();
		// Recommended delay for new reading
		delay(2);
	}

	// Reading data (CALIBRATION_MEASURES) times adding its values to "long" vars
	long sumAccX = 0;
	long sumAccY = 0;
	long sumAccZ = 0;
	long sumGyroX = 0;
	long sumGyroY = 0;
	long sumGyroZ = 0;
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
	preOffAccZ = (ACCEL_TRANSFORMATION_NUMBER - (sumAccZ / CALIBRATION_MEASURES)) / ACCEL_PREOFFSET_MAGIC_NUMBER;
	preOffGyroX = (sumGyroX / CALIBRATION_MEASURES) / GYRO_PREOFFSET_MAGIC_NUMBER;
	preOffGyroY = (sumGyroY / CALIBRATION_MEASURES) / GYRO_PREOFFSET_MAGIC_NUMBER;
	preOffGyroZ = (sumGyroZ / CALIBRATION_MEASURES) / GYRO_PREOFFSET_MAGIC_NUMBER;

	long loopCount = 0;
	bool calibrated = false;
	// Checking if readings are on the deadzone. If they are not, fix them
	while (true) {

		// Control variables
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
		sumAccZ = (ACCEL_TRANSFORMATION_NUMBER - (sumAccZ / CALIBRATION_MEASURES));
		sumGyroX = (sumGyroX / CALIBRATION_MEASURES);
		sumGyroY = (sumGyroY / CALIBRATION_MEASURES);
		sumGyroZ = (sumGyroZ / CALIBRATION_MEASURES);

		String notConverged = "";

		// Checking if readings are on the deadzone and, eventually, fixing preOffsets
		if (abs(sumAccX) <= accelDeadzoneThreshold) ready++;
		else {
			notConverged.concat("AccX");
			preOffAccX = preOffAccX - sumAccX / accelDeadzoneThreshold;
		}

		if (abs(sumAccY) <= accelDeadzoneThreshold) ready++;
		else {
			notConverged.concat(" AccY");
			preOffAccY = preOffAccY - sumAccY / accelDeadzoneThreshold;
		}

		if (abs(ACCEL_TRANSFORMATION_NUMBER - sumAccZ) <= accelDeadzoneThreshold) ready++;
		else {
			notConverged.concat(" AccZ");
			preOffAccZ = preOffAccZ - (ACCEL_TRANSFORMATION_NUMBER - sumAccZ) * (1 / accelDeadzoneThreshold);
		}

		if (abs(sumGyroX) <= gyroDeadzoneThreshold) ready++;
		else {
			notConverged.concat(" GyX");
			preOffGyroX = preOffGyroX - sumGyroX / (gyroDeadzoneThreshold + 1);
		}

		if (abs(sumGyroY) <= gyroDeadzoneThreshold) ready++;
		else {
			notConverged.concat(" GyY");
			preOffGyroY = preOffGyroY - sumGyroY / (gyroDeadzoneThreshold + 1);
		}

		if (abs(sumGyroZ) <= gyroDeadzoneThreshold) ready++;
		else {
			notConverged.concat(" GyZ");
			preOffGyroZ = preOffGyroZ - sumGyroZ / (gyroDeadzoneThreshold + 1);
		}

		loopCount++;

		// Checking if everything's ready
		if (console) {
			Serial.print (loopCount);
			Serial.print (" loops / ");
			Serial.print (ready);
			Serial.print (" axis calibrated. Missing: ");
			Serial.println (notConverged);
		}
		if (ready == 6) {
			calibrated = true;
			break;
		}
		if (loopCount >= DEADZONE_ATTEMPTS) break;
	}

	if (calibrated == false) {
		this->Calibrate();
	}
	else {

		// Setting the offsets
		this->SetAccOffsets(preOffAccX, preOffAccY, preOffAccZ);
		this->SetGyroOffsets(preOffGyroX, preOffGyroY, preOffGyroZ);

		// Executing one loop
		this->Execute();

		// Setting angle as accAngle
		angX = this->GetAngAccX();
		angY = this->GetAngAccY();
		angZ = this->GetAngAccZ();
	}
}

/*
 *	Set function for gyro offsets
 */
void MPU6050::SetGyroOffsets (float x, float y, float z) {

	// Setting offsets
	gyroXOffset = x;
	gyroYOffset = y;
	gyroZOffset = z;
}

/*
 *	Set function for accel offsets
 */
void MPU6050::SetAccOffsets (float x, float y, float z) {

	// Setting offsets
	accXOffset = x;
	accYOffset = y;
	accZOffset = z;
}

/*
 *	Set function for the acc coefficient on complementary filter
 */
void MPU6050::SetFilterAccCoeff (float coeff) {

	// Setting coefficient
	filterAccelCoeff = coeff;
}

/*
 *	Set function for the gyro coefficient on complementary filter
 */
void MPU6050::SetFilterGyroCoeff (float coeff) {

	// Setting coefficient
	filterGyroCoeff = coeff;
}

/*
 *	Set function for the accel deadzone
 */
void MPU6050::SetAccelDeadzone (float deadzone) {

	// Setting deadzone
	accelDeadzone = deadzone;
	accelDeadzoneThreshold = accelDeadzone * ACCEL_TRANSFORMATION_NUMBER;
}

/*
 *	Set function for the accel deadzone
 */
void MPU6050::SetGyroDeadzone (float deadzone) {

	// Setting deadzone
	gyroDeadzone = deadzone;
	gyroDeadzoneThreshold = gyroDeadzone * GYRO_TRANSFORMATION_NUMBER;
}