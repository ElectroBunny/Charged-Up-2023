// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Commands.ArcadeDrive;
import frc.robot.Subsystems.DriveTrain;

public class RobotContainer {

  DriveTrain driveTrain = new DriveTrain();

  public RobotContainer() {
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



  }
}


