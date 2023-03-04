// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Telescope;

public class moveTeleToPos extends CommandBase {
  private Telescope innerTele;
  private double setpoint;

  /** Creates a new moveTeleToPos. */
  public moveTeleToPos(Telescope outerTele, double length) {
    this.innerTele = outerTele;
    this.setpoint = length;
    addRequirements(this.innerTele);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.innerTele.setSetpointLength(this.setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.innerTele.moveTeleToLength();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    innerTele.stopTele();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.innerTele.teleAtSetPoint();
  }
}
