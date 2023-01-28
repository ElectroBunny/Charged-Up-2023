// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase {
  private WPI_VictorSPX armRightMotor;
  private WPI_VictorSPX armLeftMotor;

  public Arm() {
    this.armRightMotor = new WPI_VictorSPX(RobotMap.ARM_RIGHT_MOTOR);
    this.armLeftMotor = new WPI_VictorSPX(RobotMap.ARM_LEFT_MOTOR);

    armLeftMotor.setNeutralMode(NeutralMode.Coast);
    armRightMotor.setNeutralMode(NeutralMode.Coast);

    armLeftMotor.setInverted(true);
    armRightMotor.setInverted(false);

    armLeftMotor.follow(armRightMotor);
  }

  public void moveArm(double armGain){
    armRightMotor.set(armGain);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
