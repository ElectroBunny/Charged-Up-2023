// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Gripper;

public class Grip extends CommandBase {
  private Gripper innerGripper;
  // private int isGrub = -1;

  public Grip(Gripper m_teleGrip) {
    innerGripper = m_teleGrip;
    addRequirements(innerGripper);
  }

  @Override
  public void initialize() {
    // if(isGrub % 2 == 0){
    //   innerGripper.gripRelease();
    // }
    // else{ 
    //   innerGripper.gripGrab();
    // }
    innerGripper.gripToggle();
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    // isGrub += 1;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
