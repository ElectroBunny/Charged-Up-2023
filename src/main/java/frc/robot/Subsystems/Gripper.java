// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Gripper extends SubsystemBase {
  //initialize solonoieds in memory
  DoubleSolenoid doublePCMRight = null;
  DoubleSolenoid doublePCMLeft = null;

  public Gripper() {
    //define solonoieds
    doublePCMRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.RIGHT_SOLENOID_FW, RobotMap.RIGHT_SOLENOID_BW);
    doublePCMLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.LEFT_SOLENOID_FW, RobotMap.LEFT_SOLENOID_BW);
    //Sets the grip to be opened when the robots starts
    doublePCMRight.set(Value.kReverse);
    doublePCMLeft.set(Value.kReverse);
  }

  // public void gripSolenoidOff() {
  //   doublePCMRight.set(Value.kOff);
  //   doublePCMLeft.set(Value.kOff);
  // }

  // public void gripGrab() {
  //   doublePCMRight.set(Value.kForward);
  //   doublePCMLeft.set(Value.kForward);
  // }


  // public void gripRelease(){
  //   doublePCMRight.set(Value.kReverse);
  //   doublePCMLeft.set(Value.kReverse);
  // }
  
  /*
   * Function that toggles between the states of the solenoid
   */
  public void gripToggle(){
    doublePCMRight.toggle();
    doublePCMLeft.toggle();
  }

  @Override
  public void periodic() {
  }
}
