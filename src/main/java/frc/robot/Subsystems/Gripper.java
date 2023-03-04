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
  // Initialize solonoieds in memory
  private DoubleSolenoid doubleSolenoidRight;
  private DoubleSolenoid doubleSolenoidLeft;

  private boolean isOpen;

  
  public Gripper() {
    
    // Define solonoieds
    doubleSolenoidRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.RIGHT_GRIPPER_SOLENOID_FW, RobotMap.RIGHT_GRIPPER_SOLENOID_BW);
    doubleSolenoidLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.LEFT_GRIPPER_SOLENOID_FW, RobotMap.LEFT_GRIPPER_SOLENOID_BW);
    
    // Sets the gripper to be opened when the robots starts
    doubleSolenoidRight.set(Value.kReverse);
    doubleSolenoidLeft.set(Value.kReverse);
    isOpen = true;
  }

  /** Used to trun off the gripper. */
  public void gripSolenoidOff() {
    doubleSolenoidRight.set(Value.kOff);
    doubleSolenoidLeft.set(Value.kOff);

  }

  /** Used to close the gripper. */
  public void gripGrab() {
    doubleSolenoidRight.set(Value.kForward);
    doubleSolenoidLeft.set(Value.kForward);
    this.isOpen = false;
  }

  /** Used to open the gripper. */
  public void gripRelease(){
    doubleSolenoidRight.set(Value.kReverse);
    doubleSolenoidLeft.set(Value.kReverse);
    this.isOpen = true;
  }
  
  /** Function that toggles between the states of the gripper. */
  public void gripToggle(){
    if (isOpen){
      this.gripGrab();
    }
    else {
      this.gripRelease();
    }
  }

  @Override
  public void periodic() {
  }
}
