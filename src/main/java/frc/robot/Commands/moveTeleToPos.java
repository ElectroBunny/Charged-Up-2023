// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.PIDCalc;
import frc.robot.Subsystems.Telescop;

public class moveTeleToPos extends CommandBase {
  private Telescop innerTele;
  private PIDCalc encoderPID;

  public moveTeleToPos(Telescop outerTele, PIDCalc outerPID, double distance) {
    this.innerTele = outerTele;
    innerTele.setSetpointLength(distance);
    this.encoderPID = outerPID;
    addRequirements(innerTele);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    innerTele.moveTeleToPos();
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
