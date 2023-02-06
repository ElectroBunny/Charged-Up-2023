// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmPID extends SubsystemBase {
  /*
   * The constructor of the pid controller, which controls the arm movement.
   */
  PIDController pid;

  public ArmPID() {
    // pid = new PIDController(kP, kI, kD);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
