// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Telescop;

public class moveTeleManually extends CommandBase {
  private Telescop innerTele;
  private boolean getOut;

  public moveTeleManually(Telescop outerTele, boolean getOut) {
    this.getOut = getOut;
    this.innerTele = outerTele;
    addRequirements(innerTele);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(this.getOut){
      innerTele.moveTeleManually(0.3);
    }
    else{
      innerTele.moveTeleManually(-0.3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    innerTele.stopTele();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
