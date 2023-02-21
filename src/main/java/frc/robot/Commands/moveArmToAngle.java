// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.PIDCalc;

public class moveArmToAngle extends CommandBase {
  private Arm innerArm;
  private PIDCalc encoderPID;

  public moveArmToAngle(Arm outerArm, PIDCalc outerPID, double angle) {
    this.innerArm = outerArm;
    this.innerArm.setSetpointAngle(angle);
    this.encoderPID = outerPID;
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
    encoderPID.resetPID();
    innerArm.resist(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("At set point:", encoderPID.atSetPoint());
    return encoderPID.atSetPoint();
  }
}
