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
  private DoubleSolenoid doubleSolenoid = null;
  public static Compressor pcmCompressor;

  
  public Gripper() {
    //Definition of the compressor
    pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    pcmCompressor.enableDigital();
    pcmCompressor.enableAnalog(-120, 120);
    pcmCompressor.enableHybrid(-120, 120);

    //define solonoieds
    doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, RobotMap.GRIPPER_SOLENOID_FW, RobotMap.GRIPPER_SOLENOID_BW);
    //Sets the grip to be opened when the robots starts
    doubleSolenoid.set(Value.kReverse);
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
    doubleSolenoid.toggle();
  }

  @Override
  public void periodic() {
  }
}
