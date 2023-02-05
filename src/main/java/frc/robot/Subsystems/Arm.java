// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase {
  //Definition of motor controllers for the Arm system
  private WPI_VictorSPX armRightMotor;
  private WPI_VictorSPX armLeftMotor;
  
  //Definition of Encoder for the Arm system
  private Encoder right_encoder;
  private Encoder left_encoder;

  private double distanceRight = 0;
  private double distanceLeft = 0;

  private boolean directionRight;
  private boolean directionLeft;


  public Arm() {
    //initialize arm motors
    this.armRightMotor = new WPI_VictorSPX(RobotMap.ARM_RIGHT_MOTOR);
    this.armLeftMotor = new WPI_VictorSPX(RobotMap.ARM_LEFT_MOTOR);

    //initialize encoder
    right_encoder = new Encoder(RobotMap.RIGHT_ENCODER_CHANNEL_A, RobotMap.RIGHT_ENCODER_CHANNEL_B, false, EncodingType.k2X);
    left_encoder = new Encoder(RobotMap.LEFT_ENCODER_CHANNEL_A, RobotMap.LEFT_ENCODER_CHANNEL_B, true, EncodingType.k2X);
    right_encoder.setDistancePerPulse(1./256.);
    left_encoder.setDistancePerPulse(1./256.);
    right_encoder.reset();
    left_encoder.reset();

    armLeftMotor.setNeutralMode(NeutralMode.Coast);
    armRightMotor.setNeutralMode(NeutralMode.Coast);

    armLeftMotor.setInverted(true);
    armRightMotor.setInverted(false);

    armLeftMotor.follow(armRightMotor);
  }

  public void moveArm(double armGain){
    armRightMotor.set(armGain);
  }

  /*
   * Functions that get information from the encoders
   */
  // public double getDistanceRight(){
  //   return right_encoder.getDistance();
  // }

  // public double getDistanceLeft(){
  //   return left_encoder.getDistance();
  // }

  public double getRateRight(){
    return right_encoder.getRate();
  }

  public double getRateLeft(){
    return left_encoder.getRate();
  }
  
  public boolean getDirectRight(){
    directionRight = right_encoder.getDirection();
    return directionRight;
  }

  public boolean getDirectLeft(){
    directionLeft = left_encoder.getDirection();
    return directionLeft;
  }

  public double getRelativeDistanceRight(){
    if(directionRight){
      distanceRight += right_encoder.getDistance();
      right_encoder.reset();
    }
    else{
      distanceRight -= right_encoder.getDistance();
      right_encoder.reset();
    }
    return distanceRight;
  }
  
  public double getRelativeDistanceLeft(){
    if(directionLeft){
      distanceLeft += left_encoder.getDistance();
      left_encoder.reset();
    }
    else{
      distanceRight -= left_encoder.getDistance();
      left_encoder.reset();
    }
    return distanceRight;
  }
  

  @Override
  public void periodic() {
  }
 
}
