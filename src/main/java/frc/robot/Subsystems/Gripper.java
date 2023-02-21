// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Gripper extends SubsystemBase {
  //initialize solonoieds in memory
  private DoubleSolenoid doubleSolenoidRight = null;
  private DoubleSolenoid doubleSolenoidLeft = null;

  public static Compressor pcmCompressor;
  private boolean isOpen;

  
  public Gripper() {
    //Definition of the compressor

    //define solonoieds
    doubleSolenoidRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.RIGHT_GRIPPER_SOLENOID_FW, RobotMap.RIGHT_GRIPPER_SOLENOID_BW);
    doubleSolenoidLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.LEFT_GRIPPER_SOLENOID_FW, RobotMap.LEFT_GRIPPER_SOLENOID_BW);
    //Sets the grip to be opened when the robots starts
    doubleSolenoidRight.set(Value.kReverse);
    doubleSolenoidLeft.set(Value.kReverse);
    isOpen = true;
  }

  public void gripSolenoidOff() {
    doubleSolenoidRight.set(Value.kOff);
    doubleSolenoidLeft.set(Value.kOff);

  }

  public void gripGrab() {
    doubleSolenoidRight.set(Value.kForward);
    doubleSolenoidLeft.set(Value.kForward);
  }


  public void gripRelease(){
    doubleSolenoidRight.set(Value.kReverse);
    doubleSolenoidLeft.set(Value.kReverse);
  }
  
  /*
   * Function that toggles between the states of the solenoid
   */
  public void gripToggle(){
    if (isOpen){
      this.gripGrab();
    }
    else {
      this.gripRelease();
    }
    isOpen = !isOpen;
  }

  @Override
  public void periodic() {
  }
}
