// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotMap;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Telescope;

public class moveTeleManually extends CommandBase {
  private Telescope innerTele;
  private double gain;
  private Arm innerArm;
  // private Arm m_arm;

  public moveTeleManually(Telescope outerTele, double gain, Arm outerArm) {
    // this.m_arm = new Arm();
    this.gain = gain;
    this.innerTele = outerTele;
    this.innerArm = outerArm;
    addRequirements(innerTele);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // // BACKUP FOR INSP
    // if ((this.innerTele.getLength() + RobotMap.TELE_MIN_LENGTH >= 117) && this.gain > 0){
    //   this.innerTele.stopTele();
    // }
    // else {
    //   innerTele.moveTeleManually(this.gain, this.innerArm.getAngle()); //this.m_arm.getAngle()
    // }

    innerTele.moveTeleManually(this.gain, this.innerArm.getAngle()); //this.m_arm.getAngle()
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
