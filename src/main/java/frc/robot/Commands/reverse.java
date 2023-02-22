// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Telescope;

public class reverse extends CommandBase {
  private Telescope innerTele;
  // private Arm m_arm;

  public reverse(Telescope outerTele) {
    // this.m_arm = new Arm();

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
    innerTele.moveTeleManually(-1); //this.m_arm.getAngle()
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