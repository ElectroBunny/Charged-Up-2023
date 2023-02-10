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
import frc.robot.Subsystems.EncoderPIDD;;


public class Arm extends SubsystemBase {
  //Definition of motor controllers for the Arm system
  private WPI_VictorSPX armRightMotor;
  private WPI_VictorSPX armLeftMotor;

  private EncoderPIDD encoderPid;
  private Encoder encoder;
  
  public double setPointAngle;

  public Arm() {
    encoder = new Encoder(RobotMap.ARM_ENCODER_CHANNEL_A, RobotMap.ARM_ENCODER_CHANNEL_B, false, EncodingType.k2X);
    encoder.setDistancePerPulse(1./2048.);
    encoder.reset(); 

    encoderPid = new EncoderPIDD(RobotMap.KP_ARM, RobotMap.KI_ARM, RobotMap.KD_ARM, RobotMap.TOLRENCE_ARM, encoder);

    //initialize arm motors
    this.armRightMotor = new WPI_VictorSPX(RobotMap.ARM_RIGHT_MOTOR);
    this.armLeftMotor = new WPI_VictorSPX(RobotMap.ARM_LEFT_MOTOR);

    setPointAngle = 0;

    armLeftMotor.setNeutralMode(NeutralMode.Coast);
    armRightMotor.setNeutralMode(NeutralMode.Coast);

    armLeftMotor.setInverted(true);
    armRightMotor.setInverted(false);

    armLeftMotor.follow(armRightMotor);
  }

  public void moveArmToAngle(){
    armRightMotor.set(encoderPid.setToAngle(this.setPointAngle));
  }

  public Encoder returnEncoder(){
    return this.encoder;
  }

  @Override
  public void periodic() {
  }
 
}
