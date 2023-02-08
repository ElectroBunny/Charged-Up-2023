// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.ArmPID;

public class moveArmToAngle extends CommandBase {
  private Arm innerArm;
  private ArmPID armPid;

  public moveArmToAngle(Arm outerArm, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.innerArm = outerArm;
    innerArm.setPointAngle = angle;
    addRequirements(innerArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    innerArm.moveArmToAngle();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armPid.resetPID();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return armPid.atSetPoint();
  }
}
