// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.ArcadeDrive;
import frc.robot.Commands.Grip;
import frc.robot.Commands.moveArmToAngle;
import frc.robot.Commands.moveGripper;
import frc.robot.Commands.reverseGripper;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.ArmPID;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Gripper;
import frc.robot.Subsystems.TeleGrip;

public class RobotContainer {

  // private final DriveTrain driveTrain;
  // private final Gripper gripper;
  private final OI m_oi;
  private final ArmPID m_armPid;
  private final TeleGrip m_teleTeleGrip;
  private final Arm m_arm;

  public RobotContainer() {
    // driveTrain = new DriveTrain();
    // gripper = new Gripper();
    m_oi = new OI();
    m_armPid =  new ArmPID();
    m_teleTeleGrip = new TeleGrip();
    m_arm = new Arm();

    // driveTrain.setDefaultCommand(new ArcadeDrive(driveTrain));
    configureButtonBindings();
  }

  public void onRobotPeriodic(){
    SmartDashboard.putNumber("Distance R", m_armPid.getDistanceRight());
    // SmartDashboard.putNumber("Distance L", m_armPid.getDistanceLeft());
    SmartDashboard.putNumber("Rate R", m_armPid.getRateRight());
    // SmartDashboard.putNumber("Rate L", m_armPid.getRateLeft());
    SmartDashboard.putNumber("Rate R", m_armPid.getRateRight());
    // SmartDashboard.putNumber("Rate L", m_armPid.getRateLeft());
    SmartDashboard.putBoolean("Direction R", m_armPid.getDirectRight());
    // SmartDashboard.putBoolean("Direction L", m_armPid.getDirectLeft());
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

    m_oi.button1.whileTrue(new moveGripper(m_teleTeleGrip));
    m_oi.button3.whileTrue(new reverseGripper(m_teleTeleGrip));

    //Gripper mode and low scoring buttons
    m_oi.button7.onTrue(new moveArmToAngle(m_arm, RobotMap.LOW_LEFT_ANGLE));
    m_oi.button8.onTrue(new moveArmToAngle(m_arm, RobotMap.LOW_RIGHT_ANGLE));

    //Cone scoring buttons(mid and high)
    m_oi.button9.onTrue(new moveArmToAngle(m_arm, RobotMap.MID_CONE_ANGLE));
    m_oi.button10.onTrue(new moveArmToAngle(m_arm, RobotMap.HIGH_CONE_ANGLE));

    //Cube scoring buttons(mid and high)
    m_oi.button11.onTrue(new moveArmToAngle(m_arm, RobotMap.MID_CUBE_ANGLE));
    m_oi.button12.onTrue(new moveArmToAngle(m_arm, RobotMap.HIGH_CUBE_ANGLE));
  }
}


