// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// The integral gain term will never add or subtract more than 0.5 from
// the total loop output
//pid.setIntegratorRange(-0.5, 0.5);

// takes a value and put him in range of (-0.5, 0.5) by propotion. This value will be used to clamp the output value of the pid and use it in the motors.
//MathUtil.clamp(pid.calculate(encoder.getDistance(), setpoint), -0.5, 0.5);

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
