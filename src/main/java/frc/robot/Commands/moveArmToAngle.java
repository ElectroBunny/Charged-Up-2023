// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class moveArmToAngle extends CommandBase {
  private Arm innerArm;
  private double setpointAngle;

  public moveArmToAngle(double angle) {
    this.innerArm = Arm.getInstance();
    this.setpointAngle = angle;
    addRequirements(innerArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.innerArm.setSetpointAngle(this.setpointAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.innerArm.moveArmToAngle();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    innerArm.resist();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("At setpoint", this.innerArm.armAtSetPoint());
    return this.innerArm.armAtSetPoint();
  }
}
