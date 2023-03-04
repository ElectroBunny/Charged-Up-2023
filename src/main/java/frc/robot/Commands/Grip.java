// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Gripper;

public class Grip extends InstantCommand {
  private Gripper innerGripper;

  public Grip(Gripper m_teleGrip) {
    innerGripper = m_teleGrip;
    addRequirements(innerGripper);
  }

  @Override
  public void initialize() {
    innerGripper.gripToggle();
  }
}
