// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.Utilities;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class MPU6050 {
  private I2C mpu;

  private byte[] data = new byte[14];

  private int accelX;
  private int accelY;
  private int accelZ;
  private int gyroX;
  private int gyroY;
  private int gyroZ;

  public MPU6050() {
    mpu = new I2C(Port.kOnboard, 0x68);
  }

  public void readSensor() {
    mpu.read(0x3B, 14, this.data);
    this.accelX = (data[0] << 8) | data[1];
    this.accelY = (data[2] << 8) | data[3];
    this.accelZ = (data[4] << 8) | data[5];
    this.gyroX = (data[8] << 8) | data[9];
    this.gyroY = (data[10] << 8) | data[11];
    this.gyroZ = (data[12] << 8) | data[13];
    // do something with the sensor data
  }

  public int getAccelX(){
    return this.accelX;
  }

  public int getAccelY(){
    return this.accelY;
  }

  public int getAccelZ(){
    return this.accelZ;
  }
  
  public int getGyroX(){
    return this.gyroX;
  }

  public int getGyroY(){
    return this.gyroY;
  }

  public int getGyroZ(){
    return this.gyroZ;
  }
}