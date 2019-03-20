/*
 *  Mandatory includes
 */
#include <Arduino.h>
#include <TinyMPU6050.h>

/*
 *  Constructing MPU-6050
 */
MPU6050 mpu (Wire);

/*
 *  Method that prints everything
 */
void PrintGets () {
  // Shows offsets
  Serial.println("--- Offsets:");
  Serial.print("AccX Offset = ");
  Serial.println(mpu.GetAccXOffset());
  Serial.print("AccY Offset = ");
  Serial.println(mpu.GetAccYOffset());
  Serial.print("AccZ Offset = ");
  Serial.println(mpu.GetAccZOffset());
  Serial.print("GyroX Offset = ");
  Serial.println(mpu.GetGyroXOffset());
  Serial.print("GyroY Offset = ");
  Serial.println(mpu.GetGyroYOffset());
  Serial.print("GyroZ Offset = ");
  Serial.println(mpu.GetGyroZOffset());
  // Shows raw data
  Serial.println("--- Raw data:");
  Serial.print("Raw AccX = ");
  Serial.println(GetRawAccX());
  Serial.print("Raw AccY = ");
  Serial.println(GetRawAccY());
  Serial.print("Raw AccZ = ");
  Serial.println(GetRawAccZ());
  Serial.print("Raw GyroX = ");
  Serial.println(GetRawGyroX());
  Serial.print("Raw GyroY = ");
  Serial.println(GetRawGyroY());
  Serial.print("Raw GyroZ = ");
  Serial.println(GetRawGyroZ());
  // Show readable data
  Serial.println("--- Readable data:");
  Serial.print("AccX = ");
  Serial.print(GetAccX());
  Serial.println(" m/s²");
  Serial.print("AccY = ");
  Serial.print(GetAccY());
  Serial.println(" m/s²");
  Serial.print("AccZ = ");
  Serial.print(GetAccZ());
  Serial.println(" m/s²");
  Serial.print("GyroX = ");
  Serial.print(GetGyroX());
  Serial.println(" degrees/second");
  Serial.print("GyroY = ");
  Serial.print(GetGyroY());
  Serial.println(" degrees/second");
  Serial.print("GyroZ = ");
  Serial.print(GetGyroZ());
  Serial.println(" degrees/second");
  // Show angles based on accelerometer only
  Serial.println("--- Accel angles:");
  Serial.print("AccelAngX = ");
  Serial.println(GetAngAccX());
  Serial.print("AccelAngY = ");
  Serial.println(GetAngAccY());
  Serial.print("AccelAngZ = ");
  Serial.println(GetAngAccZ());
  // Show angles based on gyroscope only
  Serial.println("--- Gyro angles:");
  Serial.print("GyroAngX = ");
  Serial.println(GetAngGyroX());
  Serial.print("GyroAngY = ");
  Serial.println(GetAngGyroY());
  Serial.print("GyroAngZ = ");
  Serial.println(GetAngGyroZ());
  // Show angles based on both gyroscope and accelerometer
  Serial.println("--- Filtered angles:");
  Serial.print("FilteredAngX = ");
  Serial.println(GetAngX());
  Serial.print("FilteredAngY = ");
  Serial.println(GetAngY());
  Serial.print("FilteredAngZ = ");
  Serial.println(GetAngZ());
  // Show filter coefficients
  Serial.println("--- Angle filter coefficients:");
  Serial.print("Accelerometer percentage = ");
  Serial.print(GetFilterAccCoeff());
  Serial.println('%');
  Serial.print("Gyroscope percentage = ");
  Serial.print(GetFilterGyroCoeff());
  Serial.println('%');
  // Show accel/gyro deadzones
  Serial.println("--- Deadzone:");
  Serial.print("Accelerometer deadzone = ");
  Serial.print(GetAccelDeadzone());
  Serial.println(" m/s²");
  Serial.print("Gyroscope deadzone = ");
  Serial.print(GetGyroDeadzone());
  Serial.println(" degrees/second");
}

/*
 *  Setup
 */
void setup() {

  // Initialization
  mpu.Initialize();

  // Calibration
  Serial.begin(9600);
  Serial.println("=====================================");
  Serial.println("Starting calibration...");
  mpu.Calibrate();
  Serial.println("Calibration complete!");
}

/*
 *  Loop
 */
void loop() {
  
  PrintGets();
  delay(30000); // 30 sec delay

}