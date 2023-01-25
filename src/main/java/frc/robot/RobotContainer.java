// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Commands.ArcadeDrive;
import frc.robot.Commands.Grip;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Gripper;

public class RobotContainer {

  private final DriveTrain driveTrain;
  private final Gripper gripper;
  private final OI m_oi;

  public RobotContainer() {
    driveTrain = new DriveTrain();
    gripper = new Gripper();
    m_oi = new OI();

    driveTrain.setDefaultCommand(new ArcadeDrive(driveTrain));
    configureButtonBindings();

  }

  public void onAutoInit(){
 

}
  public void onAutoPeriodic(){
  }

  public void onTeleopInit() {
  }

    

  public void onTeleopPeriodic(){

  }


  private void configureButtonBindings() {
    /**Linking between the buttons, that defined in oi.java, to commands. */
    // m_oi.button3.whenPressed(new Grip(gripper));
    m_oi.button3.onTrue(new Grip(gripper));

  }
}


