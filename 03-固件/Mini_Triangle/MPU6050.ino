#include <Wire.h>
#include "Kalman.h"> // Source: https://github.com/TKJElectronics/KalmanFilter
#define gyroX_OFF 0 

Kalman kalmanX;
double accY, accZ;    //加速度        
double gyroX, GyroX_lowout;//角速度

double kalAngleX;     //Calculated angle using a Kalman filter 卡尔曼滤波计算X轴角度
double gyroXangle;    // Angle calculate using the gyro only
uint8_t i2cData[14];  //Buffer for I2C data
uint32_t Atimer;      //计算间隔时间dt，计算角度
double pitch,last_pitch;    //
double acc2rotation(double x, double y);

/****************************************************************
        kalman mpu6050 init
****************************************************************/ 
void Mpu6050init(){
  Wire.begin();
  i2cData[0] = 7;                               // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00;                            // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00;                            // Set Gyro Full Scale Range to ±500deg/s
  i2cData[3] = 0x00;                            // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false));    // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true));           // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68)
  { // Read "WHO_AM_I" register
    Serial.print(F("Error reading MPU6050 sensor"));
    while (1);
  }
  delay(1000); // 延时等待传感器稳定
    
  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 14));
  accY  = (int16_t)((i2cData[2] << 8)  | i2cData[3]);
  accZ  = (int16_t)((i2cData[4] << 8)  | i2cData[5]);
  
  double pitch = acc2rotation(accY, accZ);
  kalmanX.setAngle(pitch);    // Set starting angle
  gyroXangle = pitch;
  Atimer = micros();
}

void GetXAngle(){
  while (i2cRead(0x3B, i2cData, 14));
  accY  = (int16_t)((i2cData[2] << 8)  | i2cData[3]);
  accZ  = (int16_t)((i2cData[4] << 8)  | i2cData[5]);
    
  gyroX = (int16_t)((i2cData[8] << 8)  | i2cData[9]);

  
  double dt = (double)(micros() - Atimer) / 1000000.0; // Calculate delta time
  Atimer = micros();
  
  pitch = acc2rotation(accY, accZ);
  gyroX = gyroX / 131.0 + gyroX_OFF; // Convert to deg/s
  
  GyroX_lowout = gyroX;
  kalAngleX = kalmanX.getAngle(pitch, GyroX_lowout, dt);
  last_pitch = pitch;
}
/* mpu6050加速度转换为角度
            acc2rotation(ax, ay)
            acc2rotation(az, ay) */
double acc2rotation(double x, double y)
{
  double tmp_kalAngleX = (atan(x / y) *57.29578);
  if (y < 0)
  {
    return (tmp_kalAngleX + 180);
  }
  else if (x < 0)
  {
    //将当前值与前值比较，当前差值大于100则认为异常
    if (!isnan(kalAngleX) && (tmp_kalAngleX + 360 - kalAngleX) > 100) {
      //isNaN () 函数可确定值是否为非数字（Not-a-Number）。 如果该值等于 NaN，则此函数返回 true。 否则返回 false
      //Serial.print("X<0"); Serial.print("\t");
      //Serial.print(tmp_kalAngleZ); Serial.print("\t");
      //Serial.print(kalAngleZ); Serial.print("\t");
      //Serial.print("\r\n");
      if (tmp_kalAngleX < 0 && kalAngleX < 0) //按键右边角
        return tmp_kalAngleX;
      else  //按键边异常处理
        return tmp_kalAngleX;
    } else
      return (tmp_kalAngleX + 360);
  }
  else
  {
    return tmp_kalAngleX;
  }
}
