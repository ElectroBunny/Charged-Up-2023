// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Subsystems.DriveTrain;

public class ArcadeDrive extends CommandBase {
  private DriveTrain innerDriver;

  private double forwardY, turnX;
  private OI oi;

  public ArcadeDrive(DriveTrain externalDriver) {
    oi = new OI();
    innerDriver = externalDriver;
    addRequirements(innerDriver);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    forwardY = turnX = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // this.reverseY = oi.getPS4RightTriggerAxis();
    // this.forwardY = oi.getPS4LeftTriggerAxis();

    // this.xAxis = oi.getPS4LeftX();

    // innerDriver.ArcadeDrive((this.forwardY - this.reverseY), this.xAxis);  
    
    this.forwardY = oi.PS4getLeftY();
    this.turnX = oi.PS4getRightX();

    innerDriver.ArcadeDrive(this.forwardY, this.turnX);  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    innerDriver.StopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
