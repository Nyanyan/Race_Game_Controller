#include<Wire.h>

#define GYRO_NOT_SENSE_OFFSET 40

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  // Configure gyroscope range to ±2000 deg/s
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x18);  // set gyroscope range to ±2000 deg/s (11 on bits 4 and 3)
  Wire.endTransmission(true);
  
  Serial.begin(115200);
  delay(500);
}

double sum_gyz = 0;

void loop(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  // Serial.print("AcX = "); Serial.print(AcX);
  // Serial.print("\tAcY = "); Serial.print(AcY);
  // Serial.print("\tAcZ = "); Serial.print(AcZ);
  // Serial.print("\tTmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  // Serial.print("\tGyX = "); Serial.print(GyX);
  // Serial.print("\tGyY = "); Serial.print(GyY);
  // Serial.print("\tGyZ = "); Serial.println(GyZ);
  double gyz_double = GyZ;
  if (abs(gyz_double) < GYRO_NOT_SENSE_OFFSET) {
    gyz_double = 0;
  } else if (gyz_double >= GYRO_NOT_SENSE_OFFSET) {
    gyz_double -= GYRO_NOT_SENSE_OFFSET;
  } else if (gyz_double <= -GYRO_NOT_SENSE_OFFSET) {
    gyz_double += GYRO_NOT_SENSE_OFFSET;
  }
  sum_gyz += gyz_double;
  int handle = sum_gyz / 10000;
  Serial.print(gyz_double);
  Serial.print('\t');
  Serial.print(sum_gyz);
  Serial.print('\t');
  Serial.println(handle);
  delay(10);
}