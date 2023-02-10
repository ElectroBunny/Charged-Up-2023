// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class TeleGrip extends SubsystemBase {
  private WPI_VictorSPX m_teleGrip;

  private Encoder encoder;
  private EncoderPIDD encoderPid;
  public double setPointDistance;

  public TeleGrip() {
    this.m_teleGrip = new WPI_VictorSPX(RobotMap.TELESCOPIC_GRIPPER);
    this.m_teleGrip.setNeutralMode(NeutralMode.Coast);

    encoder = new Encoder(RobotMap.TELE_ENCODER_CHANNEL_A, RobotMap.TELE_ENCODER_CHANNEL_B, false, EncodingType.k2X);
    encoder.setDistancePerPulse(1./2048.);
    encoder.reset(); 

    encoderPid = new EncoderPIDD(RobotMap.KP_TELE, RobotMap.KI_TELE, RobotMap.KD_TELE, RobotMap.TOLRENCE_TELE, encoder);

  }

  public void moveTeleToDistance(){
    m_teleGrip.set(encoderPid.setToDistance(this.setPointDistance));
  }

  public Encoder returnEncoder(){
    return this.encoder;
  }

  @Override
  public void periodic() {
    
  }
}