// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.Subsystems.Arm;

public class checkArm extends CommandBase {
  private double yAxis = 0;
  private OI oi = new OI();
  private Arm innerArm;

  public checkArm(Arm outerArm) {
    this.innerArm = outerArm;
    addRequirements(innerArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    yAxis = oi.getJoystickRawAxis(RobotMap.Y_AXIS_PORT);
    innerArm.checkArm(yAxis);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    innerArm.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
