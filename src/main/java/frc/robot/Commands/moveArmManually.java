// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.PIDCalc;

public class moveArmManually extends CommandBase {
  private Arm innerArm;
  private PIDCalc encoderPID;
  private int movementDirection; //1 - up, -1 - down
  
  public moveArmManually(Arm outerArm, PIDCalc outerPID, int direction) {
    this.movementDirection = direction;
    this.encoderPID = outerPID;
    this.innerArm = outerArm;
    addRequirements(innerArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    innerArm.moveArmManually(this.movementDirection);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    encoderPID.resetPID();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return encoderPID.atSetPoint();
  }
}
