// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class TeleGrip extends SubsystemBase {
  private WPI_VictorSPX m_teleGrip;

  public TeleGrip() {
    this.m_teleGrip = new WPI_VictorSPX(RobotMap.TELESCOPIC_GRIPPER);
    this.m_teleGrip.setNeutralMode(NeutralMode.Coast);
  }

  public void moveGripper(double gripperGain){
    m_teleGrip.set(gripperGain);
  }

  @Override
  public void periodic() {
    
  }
}
