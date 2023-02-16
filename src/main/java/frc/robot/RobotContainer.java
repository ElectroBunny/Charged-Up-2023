// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.ArcadeDrive;
import frc.robot.Commands.Grip;
import frc.robot.Commands.checkArm;
import frc.robot.Commands.moveArmManually;
import frc.robot.Commands.moveArmToAngle;
import frc.robot.Commands.moveTeleManually;
import frc.robot.Commands.moveTeleToPos;
import frc.robot.Commands.reverseArm;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Gripper;
import frc.robot.Subsystems.PIDCalc;
import frc.robot.Subsystems.Telescop;


public class RobotContainer {

  private final DriveTrain driveTrain;
  // private final Gripper gripper;
  private final OI m_oi;
  // private final PIDCalc m_armPid;
  // private final PIDCalc m_telePid;
  // private final Telescop m_teleGrip;
  private final Arm m_arm;
  private final Gripper m_gripper;

  private double startTime;
  private double delta_time;


  public RobotContainer() {
    driveTrain = new DriveTrain();
    m_gripper = new Gripper();
    m_oi = new OI();
    
    m_arm = new Arm();
    // m_armPid = new PIDCalc(RobotMap.KP_ARM, RobotMap.KI_ARM, RobotMap.KD_ARM, RobotMap.TOLRENCE_ARM);
    
    // m_teleGrip = new Telescop();
    // m_telePid = new PIDCalc(RobotMap.KP_TELE, RobotMap.KI_TELE, RobotMap.KD_TELE, RobotMap.TOLRENCE_TELE);

    // driveTrain.setDefaultCommand(new ArcadeDrive(driveTrain));

    configureButtonBindings();
  }

  public void onRobotPeriodic(){
    // SmartDashboard.putNumber("Distance R", m_armPid.getDistanceRight());
    // SmartDashboard.putNumber("Distance L", m_armPid.getDistanceLeft());
    // SmartDashboard.putNumber("Rate R", m_armPid.getRateRight());
    // SmartDashboard.putNumber("Rate L", m_armPid.getRateLeft());
    // SmartDashboard.putNumber("Rate R", m_armPid.getRateRight());
    // SmartDashboard.putNumber("Rate L", m_armPid.getRateLeft());
    // SmartDashboard.putBoolean("Direction R", m_armPid.getDirectRight());
    // SmartDashboard.putBoolean("Direction L", m_armPid.getDirectLeft());
  }

  public void onAutoInit(){
    this.startTime = Timer.getFPGATimestamp();
  }
  
  /**
  * Sets the tool tip text.
  * @param armSetPoint the angle that you want the arm to move to
  */
  // public void onSimpleAuto(double armSetPoint){
  //   //Sets the setpoint of the PID calculations
  //   m_arm.setSetpointAngle(armSetPoint);

  //   delta_time = Timer.getFPGATimestamp() - startTime;
  //   if(!m_armPid.atSetPoint()){
  //     m_arm.moveArmToAngle();
  //   }

  //   else if(!m_telePid.atSetPoint()){
  //     m_teleGrip.moveTeleToPos();
  //   }

  //   else if(delta_time < RobotMap.SIMPLE_AUTO_DRIVE_TIME){
  //     driveTrain.ArcadeDrive(0.6, 0);
  //   }

  // }

  public void onAutoMid(){
  }


  // public void onTeleopInit() {
  //   driveTrain.ArcadeDrive(0, 0);
  //   m_armPid.resetPID();
  //   m_telePid.resetPID();
  // }

    

  public void onTeleopPeriodic(){

  }


  private void configureButtonBindings() {
    /**Linking between the buttons, that defined in oi.java, to commands. */

    m_oi.button1.onTrue(new Grip(m_gripper));

    // m_oi.button3.onTrue(new moveArmManually(m_arm, m_armPid, +1));
    // m_oi.button4.onTrue(new moveArmManually(m_arm, m_armPid, -1));


    m_oi.povbutton1.whileTrue(new checkArm(m_arm));
    m_oi.povbutton2.whileTrue(new reverseArm(m_arm));

    // //Gripper mode and low scoring buttons
    // m_oi.button7.onTrue(new moveArmToAngle(m_arm, m_armPid, RobotMap.LOW_FRONT_ANGLE)
    //   .andThen(new moveTeleToPos(m_teleGrip, m_telePid, RobotMap.TELE_LOW_DISTANCE_FRONT))
    //   .andThen(new Grip(m_gripper))
    //   .andThen(new moveTeleToPos(m_teleGrip, m_telePid, 0)));

    // m_oi.button8.onTrue(new moveArmToAngle(m_arm, m_armPid, RobotMap.LOW_BACK_ANGLE)
    //   .andThen(new moveTeleToPos(m_teleGrip, m_telePid, RobotMap.TELE_LOW_DISTANCE_BACK))
    //   .andThen(new Grip(m_gripper))
    //   .andThen(new moveTeleToPos(m_teleGrip, m_telePid, 0)));

    // //Cone scoring buttons(mid and high)
    // m_oi.button9.onTrue(new moveArmToAngle(m_arm, m_armPid, RobotMap.MID_CONE_ANGLE)
    //   .andThen(new moveTeleToPos(m_teleGrip, m_telePid, RobotMap.TELE_MID_DISTANCE_CONE))
    //   .andThen(new Grip(m_gripper))
    //   .andThen(new moveTeleToPos(m_teleGrip, m_telePid, 0)));
      
    // m_oi.button10.onTrue(new moveArmToAngle(m_arm, m_armPid, RobotMap.HIGH_CONE_ANGLE)
    //   .andThen(new moveTeleToPos(m_teleGrip, m_telePid, RobotMap.TELE_HIGH_DISTANCE_CONE))
    //   .andThen(new Grip(m_gripper))
    //   .andThen(new moveTeleToPos(m_teleGrip, m_telePid, 0)));

    // //Cube scoring buttons(mid and high)
    // m_oi.button11.onTrue(new moveArmToAngle(m_arm, m_armPid, RobotMap.MID_CUBE_ANGLE)
    //   .andThen(new moveTeleToPos(m_teleGrip, m_telePid, RobotMap.TELE_MID_DISTANCE_CUBE))
    //   .andThen(new Grip(m_gripper))
    //   .andThen(new moveTeleToPos(m_teleGrip, m_telePid, 0)));
    
    // m_oi.button12.onTrue(new moveArmToAngle(m_arm, m_armPid, RobotMap.HIGH_CUBE_ANGLE)
    //   .andThen(new moveTeleToPos(m_teleGrip, m_telePid, RobotMap.TELE_HIGH_DISTANCE_CUBE))
    //   .andThen(new Grip(m_gripper))
    //   .andThen(new moveTeleToPos(m_teleGrip, m_telePid, 0)));
  }
}


//movetele to distense 
//rel