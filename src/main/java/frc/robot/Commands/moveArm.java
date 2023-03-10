// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class moveArm extends CommandBase {
  private Arm innerArm;
  private double setpoint;

  public moveArm(double angle) {
    this.innerArm = Arm.getInstance();
    this.setpoint = angle;
    addRequirements(innerArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.innerArm.setSetpointAngle(this.setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.innerArm.move_arm();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    innerArm.resist();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.innerArm.armAtSetPoint();
  }
}
