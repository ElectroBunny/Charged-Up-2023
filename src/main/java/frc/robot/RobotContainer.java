// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Period;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.AxisCamera;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.ArcadeDrive;
import frc.robot.Commands.ChangeDriveVelocity;
import frc.robot.Commands.Grip;
import frc.robot.Commands.moveArmManually;
import frc.robot.Commands.moveArmToAngle;
import frc.robot.Commands.moveTeleManually;
// import frc.robot.Commands.moveTeleToPos;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Gripper;
import frc.robot.Subsystems.PIDCalc;
import frc.robot.Subsystems.Telescope;
import frc.robot.Utilities.MPU6050;


public class RobotContainer {

  private final DriveTrain driveTrain;
  private final OI m_oi;
  private final PIDCalc m_armPid;
  // private final PIDCalc m_telePid;
  private final Telescope m_teleGrip;
  private final Arm m_arm;
  private final Gripper m_gripper;

  private double startTime;
  private double delta_time;

  public static Compressor pcmCompressor;

  private MPU6050 mpu;

  CvSink cvSink;
  CvSource outputStream;

  public RobotContainer() {
    driveTrain = new DriveTrain();
    m_gripper = new Gripper();
    m_oi = new OI();
    m_arm = new Arm();
    m_teleGrip = new Telescope();
    m_armPid = new PIDCalc(RobotMap.KP_ARM, RobotMap.KI_ARM, RobotMap.KD_ARM, RobotMap.TOLRENCE_ARM);
    // m_telePid = new PIDCalc(RobotMap.KP_TELE, RobotMap.KI_TELE, RobotMap.KD_TELE, RobotMap.TOLRENCE_TELE);


    //might cause a problem
    pcmCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
    pcmCompressor.enableDigital();
    // pcmCompressor.enableAnalog(0, 120);

    mpu = new MPU6050();

    configureButtonBindings();
  }

  public void startCamera(){
    CameraServer.startAutomaticCapture();
    cvSink = CameraServer.getVideo();
    outputStream = CameraServer.putVideo("Blur", 640, 480);
  }

  public void controlCameraViewSide(){
    if(m_arm.getAngle() <= 180){

    }
    else{

    }
  }

  public void onRobotPeriodic(){
    SmartDashboard.putNumber("Arm angle", m_arm.getAngle());
    SmartDashboard.putNumber("Telescope length", m_teleGrip.getLength());

    SmartDashboard.putNumber("Time", Timer.getFPGATimestamp());

    SmartDashboard.putNumber("AccelX", mpu.getAccelX());
    SmartDashboard.putNumber("AccelY", mpu.getAccelY());
    SmartDashboard.putNumber("AccelZ", mpu.getAccelZ());
    SmartDashboard.putNumber("GyroX", mpu.getGyroX());
    SmartDashboard.putNumber("GyroY", mpu.getGyroY());
    SmartDashboard.putNumber("GyroZ", mpu.getGyroZ());
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

  public void onDisabledInit(){
    m_arm.changeToCoast();
  }


  private void configureButtonBindings() {
    /**Linking between the buttons, that defined in oi.java, to commands. */
    
    driveTrain.setDefaultCommand(new ArcadeDrive(driveTrain));
    // m_oi.A.onTrue(new ChangeDriveVelocity(driveTrain));

    m_oi.button1.onTrue(new Grip(m_gripper));

    // m_oi.povbutton1.whileTrue(new moveTeleManually(m_teleGrip, 1));
    // m_oi.povbutton2.whileTrue(new moveTeleManually(m_teleGrip, -1));


    m_oi.button2.whileTrue(new moveArmManually(m_arm));

    //CONE BUTTONS 7,9

    m_oi.button7.onTrue(new moveArmToAngle(m_arm, m_armPid, RobotMap.HIGH_CONE_ANGLE));
    m_oi.button9.onTrue(new moveArmToAngle(m_arm, m_armPid, RobotMap.MID_CONE_ANGLE));

    //CUBE BUTTONS 8,10
    m_oi.button7.onTrue(new moveArmToAngle(m_arm, m_armPid, RobotMap.HIGH_CUBE_ANGLE));
    m_oi.button9.onTrue(new moveArmToAngle(m_arm, m_armPid, RobotMap.MID_CUBE_ANGLE));

    //low (back and front)
    m_oi.button11.onTrue(new moveArmToAngle(m_arm, m_armPid, RobotMap.LOW_BACK_ANGLE));
    m_oi.button12.onTrue(new moveArmToAngle(m_arm, m_armPid, RobotMap.LOW_FRONT_ANGLE));
  }
}