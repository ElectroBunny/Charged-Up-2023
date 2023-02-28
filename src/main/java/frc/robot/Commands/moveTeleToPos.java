// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Telescope;
import frc.robot.Subsystems.PIDCalc;

public class moveTeleToPos extends CommandBase {
  private Telescope innerTele;
  private Arm innerArm;
  private PIDCalc encoderPID;
  private double setpoint;

  /** Creates a new moveTeleToPos. */
  public moveTeleToPos(Telescope outerTele, Arm outerArm, PIDCalc outerPid, double length) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.innerTele = outerTele;
    this.innerArm = outerArm;
    this.encoderPID = outerPid;
    this.setpoint = length;
    SmartDashboard.putNumber("Length setpoint", length);
    addRequirements(this.innerTele);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.encoderPID.setSetpoint(this.setpoint);
    this.innerTele.setSetpointLength(this.setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.encoderPID.setInput(this.innerTele.getLength() + RobotMap.TELE_MIN_LENGTH);
    this.innerTele.moveTeleToLength();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    encoderPID.resetPID();
    innerTele.stopTele();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("At set point:", encoderPID.atSetPoint());
    return this.encoderPID.atSetPoint();
    // return false;
  }
}
