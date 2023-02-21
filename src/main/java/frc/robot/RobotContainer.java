// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Period;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.ArcadeDrive;
import frc.robot.Commands.Grip;
import frc.robot.Commands.Release;
import frc.robot.Commands.moveArmManually;
import frc.robot.Commands.moveArmToAngle;
import frc.robot.Commands.moveTeleManually;
import frc.robot.Commands.moveTeleToPos;
import frc.robot.Commands.resist;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Gripper;
import frc.robot.Subsystems.PIDCalc;
import frc.robot.Subsystems.Telescope;


public class RobotContainer {

  private final DriveTrain driveTrain;
  private final OI m_oi;
  private final PIDCalc m_armPid;
  // private final PIDCalc m_telePid;
  private final Telescope m_teleGrip;
  private final Arm m_arm;
  // private final Gripper m_gripper;

  private double startTime;
  private double delta_time;

  public static Compressor pcmCompressor;


  public RobotContainer() {
    driveTrain = new DriveTrain();
    // m_gripper = new Gripper();
    m_oi = new OI();
    m_arm = new Arm();
    m_teleGrip = new Telescope();
    m_armPid = new PIDCalc(RobotMap.KP_ARM, RobotMap.KI_ARM, RobotMap.KD_ARM, RobotMap.TOLRENCE_ARM);
    // m_telePid = new PIDCalc(RobotMap.KP_TELE, RobotMap.KI_TELE, RobotMap.KD_TELE, RobotMap.TOLRENCE_TELE);


    //might cause a problem
    pcmCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
    pcmCompressor.enableDigital();
    // pcmCompressor.enableAnalog(0, 120);

    configureButtonBindings();
  }

  public void onRobotPeriodic(){
    SmartDashboard.putNumber("Arm angle", m_arm.getAngle());
    SmartDashboard.putNumber("Telescope length", m_teleGrip.getLength());
    SmartDashboard.putNumber("Time", Timer.getFPGATimestamp());
    System.out.println(m_arm.getAngle());
  }

  public void onAutoInit(){
    //Saves the time when the autonomus started.
    this.startTime = Timer.getFPGATimestamp();
  }
  
  /**
  * Sets the tool tip text.
  * @param armSetPoint the angle that you want the arm to move to (autonomus)
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
    
    driveTrain.setDefaultCommand(new ArcadeDrive(driveTrain));

    // m_oi.button1.onTrue(new Grip(m_gripper));

    // m_oi.povbutton1.whileTrue(new moveTeleManually(m_teleGrip, RobotMap.TELESCOPE_GAIN));
    // m_oi.povbutton2.whileTrue(new moveTeleManually(m_teleGrip, 0 - RobotMap.TELESCOPE_GAIN));


    // m_oi.button7.whileTrue(new resist(m_arm));
    m_oi.button2.whileTrue(new moveArmManually(m_arm));

    m_oi.button3.onTrue(new moveArmToAngle(m_arm, m_armPid, 180));

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