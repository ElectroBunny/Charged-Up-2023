// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.ArcadeDrive;
import frc.robot.Commands.Grip;
import frc.robot.Commands.moveGripper;
import frc.robot.Commands.reverseGripper;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Gripper;
import frc.robot.Subsystems.TeleGrip;

public class RobotContainer {

  // private final DriveTrain driveTrain;
  // private final Gripper gripper;
  private final OI m_oi;
  private final Arm m_arm;
  private final TeleGrip m_teleTeleGrip;

  public RobotContainer() {
    // driveTrain = new DriveTrain();
    // gripper = new Gripper();
    m_oi = new OI();
    m_arm =  new Arm();
    m_teleTeleGrip = new TeleGrip();

    // driveTrain.setDefaultCommand(new ArcadeDrive(driveTrain));
    configureButtonBindings();
  }

  public void onRobotPeriodic(){
    SmartDashboard.putNumber("Distance R", m_arm.getDistanceRight());
    // SmartDashboard.putNumber("Distance L", m_arm.getDistanceLeft());
    SmartDashboard.putNumber("Rate R", m_arm.getRateRight());
    // SmartDashboard.putNumber("Rate L", m_arm.getRateLeft());
    SmartDashboard.putNumber("Rate R", m_arm.getRateRight());
    // SmartDashboard.putNumber("Rate L", m_arm.getRateLeft());
    SmartDashboard.putBoolean("Direction R", m_arm.getDirectRight());
    // SmartDashboard.putBoolean("Direction L", m_arm.getDirectLeft());
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
    // m_oi.button3.onTrue(new Grip(gripper));
    // m_oi.button1.whileHeld(new moveGripper(m_teleTeleGrip));
    // m_oi.button1.whenPressed(new moveGripper(m_teleTeleGrip));
    m_oi.button1.whileTrue(new moveGripper(m_teleTeleGrip));
    m_oi.button3.whileTrue(new reverseGripper(m_teleTeleGrip));

  }
}


