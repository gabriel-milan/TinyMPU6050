/*
 *	Register map for the MPU6050 available at: https://www.invensense.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 */
#include "TinyMPU6050.h"

/*
 *	Constructor
 */
MPU6050::MPU6050(TwoWire &w,int i2cAddress) {

	// Setting I²C stuff
	wire = &w;
	if(i2cAddress == MPU6050_ADDRESS_LOW || i2cAddress == MPU6050_ADDRESS_HIGH){
		address = i2cAddress;
	} else{
		address = MPU6050_ADDRESS_LOW;
	}
	
}

/*
 *	Initialization method
 */

#ifdef ESP8266
void MPU6050::Initialize (int sda, int scl) {

	// Beginning Wire
	wire->begin(sda, scl);
	this->BaseInititalize();
}
#endif

/*
 *	Wrap an angle to the interval [-180, +180]
 *  --> Suggestion by edgar-bonet at https://github.com/gabriel-milan/TinyMPU6050/issues/6
 */
static float wrap(float angle)
{
	while (angle > +180) angle -= 360;
	while (angle < -180) angle += 360;
	return angle;
}

/*
 *	Compute the weighted average of a (with weight wa)
 *	and b (weight wb), treating both a and b as angles.
 *	It is assumed the sum of the weights is 1.
 */
static float angle_average(float wa, float a, float wb, float b)
{
	return wrap(wa * a + wb * (a + wrap(b-a)));
}

void MPU6050::Initialize () {

	// Beginning Wire
	wire->begin();
	this->BaseInititalize();
}

void MPU6050::BaseInititalize () {
	// Setting attributes with default values
	filterAccelCoeff = DEFAULT_ACCEL_COEFF;
	filterGyroCoeff = DEFAULT_GYRO_COEFF;

	// Setting sample rate divider
	this->RegisterWrite(MPU6050_SMPLRT_DIV, 0x00);

	// Setting frame synchronization and the digital low-pass filter
	this->RegisterWrite(MPU6050_CONFIG, 0x00);

	// Setting gyro self-test and full scale range
	this->RegisterWrite(MPU6050_GYRO_CONFIG, 0x08);

	// Setting accelerometer self-test and full scale range
	this->RegisterWrite(MPU6050_ACCEL_CONFIG, 0x00);

	// Waking up MPU6050
	this->RegisterWrite(MPU6050_PWR_MGMT_1, 0x01);

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
void MPU6050::Execute () {

	// Updating raw data before processing it
	this->UpdateRawAccel();
	this->UpdateRawGyro();

	// Computing readable accel/gyro data
	accX = (float)(rawAccX) * ACCEL_TRANSFORMATION_NUMBER;
	accY = (float)(rawAccY) * ACCEL_TRANSFORMATION_NUMBER;
	accZ = (float)(rawAccZ) * ACCEL_TRANSFORMATION_NUMBER;
	gyroX = (float)(rawGyroX - gyroXOffset) * GYRO_TRANSFORMATION_NUMBER;
	gyroY = (float)(rawGyroY - gyroYOffset) * GYRO_TRANSFORMATION_NUMBER;
	gyroZ = (float)(rawGyroZ - gyroZOffset) * GYRO_TRANSFORMATION_NUMBER;

	// Computing accel angles
	angAccX = wrap((atan2(accY, sqrt(accZ * accZ + accX * accX))) * RAD_TO_DEG);
	angAccY = wrap((-atan2(accX, sqrt(accZ * accZ + accY * accY))) * RAD_TO_DEG);

	// Computing gyro angles
	dt = (millis() - intervalStart) * 0.001;
	angGyroX = wrap(angGyroX + gyroX * dt);
	angGyroY = wrap(angGyroY + gyroY * dt);
	angGyroZ = wrap(angGyroZ + gyroZ * dt);

	// Computing complementary filter angles
	angX = angle_average(filterAccelCoeff, angAccX, filterGyroCoeff, angX + gyroX * dt);
	angY = angle_average(filterAccelCoeff, angAccY, filterGyroCoeff, angY + gyroY * dt);
	angZ = angGyroZ;

	// Reseting the integration timer
	intervalStart = millis();
}

/*
 *	Raw accel data update method
 */
void MPU6050::UpdateRawAccel () {

	// Beginning transmission for MPU6050
	wire->beginTransmission(address);

	// Accessing accel data registers
	wire->write(MPU6050_ACCEL_XOUT_H);
	wire->endTransmission(false);

	// Requesting accel data
	wire->requestFrom(address, 6, (int) true);

	// Storing raw accel data
	rawAccX = wire->read() << 8;
	rawAccX |= wire->read();

	rawAccY = wire->read() << 8;
	rawAccY |= wire->read();

	rawAccZ = wire->read() << 8;
	rawAccZ |= wire->read();
}

/*
 *	Raw gyro data update method
 */
void MPU6050::UpdateRawGyro () {

	// Beginning transmission for MPU6050
	wire->beginTransmission(address);

	// Accessing gyro data registers
	wire->write(MPU6050_GYRO_XOUT_H);
	wire->endTransmission(false);

	// Requesting gyro data
	wire->requestFrom(address, 6, (int) true);

	// Storing raw gyro data
	rawGyroX = wire->read() << 8;
	rawGyroX |= wire->read();

	rawGyroY = wire->read() << 8;
	rawGyroY |= wire->read();

	rawGyroZ = wire->read() << 8;
	rawGyroZ |= wire->read();
}

/*
 *	Register write method
 */
void MPU6050::RegisterWrite (byte registerAddress, byte data) {

	// Starting transmission for MPU6050
	wire->beginTransmission(address);

	// Accessing register
	wire->write(registerAddress);

	// Writing data
	wire->write(data);

	// Closing transmission
	wire->endTransmission();
}

/*
 *	MPU-6050 calibration method inspired by https://forum.arduino.cc/index.php?topic=619352.0
 */
void MPU6050::Calibrate () {

	// Values from method
	double kPGyro = 0.6;
	double kIGyro = 50;
	double kPAccel = 0.15;
	double kIAccel = 8;
	float MPUOffsets[6];
	static float ITerm[6] = {0, 0, 0, 0, 0, 0};
	static float ITermReadings[10][6];
	int ReadingDetailTime = 1000;
	int Tries = 0;
	uint32_t ITermLastSum;
	int ITermSameCtr;
	int ITermSameCtr2;
	int ITermSameCtrMax = 100;
	int TestCtr = 0;
	int ITermSameCtrLimit1 = 5;
	int ITermSameCtrLimit2 = 10  ;
	unsigned long DetailsTimer;

	// Discarding few measures
	for (int i = 0; i < DISCARDED_MEASURES; i++) {
		this->UpdateRawAccel();
		this->UpdateRawGyro();
		delay(2);
	}
	this -> SetAccelOffsets(0, 0, 0);
	this -> SetGyroOffsets (0, 0, 0);

	// PI Variables
	float Error[6];
	float PTerm[6];
	float DTerm[6] = {0, 0, 0, 0, 0, 0};
	float g[3];
	float a[3];
	int ax, ay, az, gx, gy, gz;
	long error;
	uint32_t ITermSum;
	DetailsTimer = millis();

	// Calibration loop
	while (1) {

		Tries++;
		this->UpdateRawAccel();
		this->UpdateRawGyro();
		ax = this->GetRawAccX();
		ay = this->GetRawAccY();
		az = this->GetRawAccZ();
		gx = this->GetRawGyroX();
		gy = this->GetRawGyroY();
		gz = this->GetRawGyroZ();
		az -= 16384; //remove Gravity

		if (Tries == 1) { // Set it close and get a new reading

			this->UpdateRawAccel();
			this->UpdateRawGyro();
			ax = this->GetRawAccX();
			ay = this->GetRawAccY();
			az = this->GetRawAccZ();
			gx = this->GetRawGyroX();
			gy = this->GetRawGyroY();
			gz = this->GetRawGyroZ();
			az -= 16384; //remove Gravity

			this->SetAccelOffsets( -(ax >> 3), -(ay >> 3), -(az >> 3) );
			this->SetGyroOffsets( -(gx >> 2), -(gy >> 2), -(gz >> 2) );

			this->UpdateRawAccel();
			this->UpdateRawGyro();
			ax = this->GetRawAccX();
			ay = this->GetRawAccY();
			az = this->GetRawAccZ();
			gx = this->GetRawGyroX();
			gy = this->GetRawGyroY();
			gz = this->GetRawGyroZ();
			az -= 16384; //remove Gravity

			// Prime PI Loop
			ITerm[0] = -(ax);
			ITerm[1] = -(ay);
			ITerm[2] = -(az);
			ITerm[3] = -(gx);
			ITerm[4] = -(gy);
			ITerm[5] = -(gz);
			DetailsTimer = millis();
		}
		else {

			a[0] = ax;
			a[1] = ay;
			a[2] = az;
			g[0] = gx;
			g[1] = gy;
			g[2] = gz;
			ITermSum = 0;

			for (int i = 0; i < 6; i++) ITermSum += abs(ITerm[i]);

			ITermSameCtr = (ITermSum == ITermLastSum) ? (ITermSameCtr + 1) : 0;
			ITermSameCtr2 = (ITermSum == ITermLastSum) ? (ITermSameCtr2 + 1) : 0;
			ITermLastSum = ITermSum;
			ITermSameCtrMax = max (ITermSameCtrMax, ITermSameCtr);

			if (((millis() - DetailsTimer) >= (ReadingDetailTime)) || (ITermSameCtr2 >= ITermSameCtrLimit2)) {

				DetailsTimer = millis();

				if ((ITermSameCtr >= ITermSameCtrLimit1) || (ITermSameCtr2 >= ITermSameCtrLimit2)) {

					ITermSameCtr = 0;

					if ((ITermSameCtr2 >= ITermSameCtrLimit2)) {

						for (int i = 0; i < 6; i++) {
							ITermReadings[TestCtr % 10][i] = ITerm[i];// % is remainder after division. This prevents us from counting over 9 (0-9) 10/10 remainder = 0
						}

						// Failed to calibrate
						return;
					}

					// Re-tune PID and run again
					ReadingDetailTime  *= .95;
					kPGyro = kPGyro * 0.5;
					kIGyro = kIGyro * 0.5;
					kPAccel = kPAccel * 0.5;
					kIAccel = kIAccel * 0.5;
					DetailsTimer = millis();
				}
				ReadingDetailTime  *= .60;
				kPGyro = kPGyro * 0.70;
				kIGyro = kIGyro * 0.70;
				kPAccel = kPAccel * 0.70;
				kIAccel = kIAccel * 0.70;
			}
			/* PI of PID Calculations */
			for (int i = 0; i < 3; i++) { // PI Calculations
				// Accellerometer
				Error[i] = 0 - a[i] ;
				PTerm[i] = kPAccel * Error[i ];
				ITerm[i] += Error[i] * 0.002 * kIAccel; // Integral term 1000 Calculations a second = 0.001
				MPUOffsets[i] = (PTerm[i] + ITerm[i] ); //Compute PID Output
				// Gyro
				Error[i + 3] = 0 - g[i];
				PTerm[i + 3] = kPGyro *  Error[i + 3];
				ITerm[i + 3] += Error[i + 3] *  0.002 * kIGyro; // Integral term 1000 Calculations a second = 0.001
				MPUOffsets[i + 3] = (PTerm[i + 3] + ITerm[i + 3]); //Compute PID Output
			}
			this->SetAccelOffsets( round(MPUOffsets[0] / 8), round(MPUOffsets[1] / 8), round(MPUOffsets[2] / 8) );
			this->SetGyroOffsets( round(MPUOffsets[3] / 4), round(MPUOffsets[4] / 4), round(MPUOffsets[5]) / 4 );
			// get another reading for next loop
		}
		delay(2);
	};

	// Success
	return;









	float sumGyroX = 0;
	float sumGyroY = 0;
	float sumGyroZ = 0;
	
	for (int i = 0; i < CALIBRATION_MEASURES; i++) {
		this->UpdateRawAccel();
		this->UpdateRawGyro();
		sumGyroX += this->GetRawGyroX();
		sumGyroY += this->GetRawGyroY();
		sumGyroZ += this->GetRawGyroZ();
		delay(2);
	}

	sumGyroX  /= CALIBRATION_MEASURES;
	sumGyroY  /= CALIBRATION_MEASURES;
	sumGyroZ  /= CALIBRATION_MEASURES;

	this->SetGyroOffsets(sumGyroX, sumGyroY, sumGyroZ);
}

/*
 *	Set function for accel offsets
 */
void MPU6050::SetAccelOffsets (float x, float y, float z) {

	// Setting offsets
	accelXOffset = x;
	accelYOffset = y;
	accelZOffset = z;
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
