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
import frc.robot.Commands.moveArm;
import frc.robot.Commands.moveArmManually;
import frc.robot.Commands.moveArmToAngle;
import frc.robot.Commands.moveTeleManually;
import frc.robot.Commands.moveTeleToPos;
import frc.robot.Commands.moveTeleToPos;
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
  private final PIDCalc m_telePid;
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
    m_armPid = new PIDCalc(RobotMap.KP_ARM, RobotMap.KI_ARM, RobotMap.KD_ARM, RobotMap.TOLRENCE_ARM);
    m_arm = new Arm(m_armPid);
    m_teleGrip = new Telescope();
    m_telePid = new PIDCalc(RobotMap.KP_TELE, RobotMap.KI_TELE, RobotMap.KD_TELE, RobotMap.TOLRENCE_TELE);


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

    // SmartDashboard.putNumber("AccelX", mpu.getAccelX());
    // SmartDashboard.putNumber("AccelY", mpu.getAccelY());
    // SmartDashboard.putNumber("AccelZ", mpu.getAccelZ());
    // SmartDashboard.putNumber("GyroX", mpu.getGyroX());
    // SmartDashboard.putNumber("GyroY", mpu.getGyroY());
    // SmartDashboard.putNumber("GyroZ", mpu.getGyroZ());
  }

  public void onAutoInit(){
    //Saves the time when the autonomus started.
    this.startTime = Timer.getFPGATimestamp();
    m_gripper.gripRelease();
  }

  public void resetEncoders(){
    m_arm.resetEn();
    m_teleGrip.resetEn();
  }
  
  /**
  * Sets the tool tip text.
  * @param armSetPoint the angle that you want the arm to move to (autonomus)
  */
  public void onSimpleAuto(double armSetpoint, double teleSetpoint, double driveTime){

    //Sets the setpoint of the PID calculations

    // command template
    // init
    // while (not finish):
    //    execute
    // end


    // close gripper
    m_gripper.gripToggle();


    // close tele to zero
    m_telePid.setSetpoint(RobotMap.TELE_MIN_LENGTH + 3);
    m_teleGrip.setSetpointLength(RobotMap.TELE_MIN_LENGTH + 3);
    while (!this.m_telePid.atSetPoint()){
      m_telePid.setInput(this.m_teleGrip.getLength() + RobotMap.TELE_MIN_LENGTH);
      m_teleGrip.moveTeleToLength();
    }
    m_telePid.resetPID();
    m_teleGrip.stopTele();


    // send arm to angle
    m_armPid.setSetpoint(armSetpoint);
    m_arm.setSetpointAngle(armSetpoint);
    while (!m_armPid.atSetPoint()){
      m_armPid.setInput(m_arm.getAngle());
      m_arm.move_arm();
    }
    m_armPid.resetPID();
    m_arm.resist(0);


    // open tele
    m_telePid.setSetpoint(teleSetpoint);
    m_teleGrip.setSetpointLength(teleSetpoint);
    while (!this.m_telePid.atSetPoint()){
      m_telePid.setInput(this.m_teleGrip.getLength() + RobotMap.TELE_MIN_LENGTH);
      m_teleGrip.moveTeleToLength();
    }
    m_telePid.resetPID();
    m_teleGrip.stopTele();


    // release gripper
    m_gripper.gripToggle();


    // close tele to zero
    m_telePid.setSetpoint(RobotMap.TELE_MIN_LENGTH + 3);
    m_teleGrip.setSetpointLength(RobotMap.TELE_MIN_LENGTH + 3);
    while (!this.m_telePid.atSetPoint()){
      m_telePid.setInput(this.m_teleGrip.getLength() + RobotMap.TELE_MIN_LENGTH);
      m_teleGrip.moveTeleToLength();
    }
    m_telePid.resetPID();
    m_teleGrip.stopTele();


    // send arm to low
    m_armPid.setSetpoint(RobotMap.LOW_BACK_ANGLE);
    m_arm.setSetpointAngle(RobotMap.LOW_BACK_ANGLE);
    while (!m_armPid.atSetPoint()){
      m_armPid.setInput(m_arm.getAngle());
      m_arm.move_arm();
    }
    m_armPid.resetPID();
    m_arm.resist(0);


    // drive forward
    this.delta_time = Timer.getFPGATimestamp() - startTime;
    if(delta_time < driveTime){
      driveTrain.ArcadeDrive(0.6, 0);
    }
  }


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

    m_oi.povbutton1.whileTrue(new moveTeleManually(m_teleGrip, 1, m_arm));
    m_oi.povbutton2.whileTrue(new moveTeleManually(m_teleGrip, -1, m_arm));


    m_oi.button2.whileTrue(new moveArmManually(m_arm, m_teleGrip));

    
    // m_oi.button3.onTrue(new moveArm(m_arm, m_armPid, 270));
    // m_oi.button4.onTrue(new moveArm(m_arm, m_armPid, 90));
    // m_oi.button5.onTrue(new moveTeleToPos(m_teleGrip, m_arm, m_armPid, 120));
    // m_oi.button6.onTrue(new moveTeleToPos(m_teleGrip, m_arm, m_armPid, 130));
    // m_oi.button5.onTrue(new moveTeleToPos(m_teleGrip, m_arm, m_telePid, 20).andThen());
    // m_oi.button3.onTrue(new moveArm(m_arm, m_armPid, RobotMap.HIGH_CONE_ANGLE));
    // m_oi.button7.onTrue(new moveArm(m_arm, m_armPid, RobotMap.HIGH_CONE_ANGLE));
    // m_oi.button9.onTrue(new moveArm(m_arm, m_armPid, RobotMap.MID_CUBE_ANGLE));

    // CONE BUTTONS 7,9
    m_oi.button7.onTrue(new moveTeleToPos(m_teleGrip, m_arm, m_telePid, RobotMap.TELE_MIN_LENGTH+3)
    .andThen(new moveArm(m_arm, m_armPid, RobotMap.HIGH_CONE_ANGLE))
    .andThen(new moveTeleToPos(m_teleGrip, m_arm, m_telePid, RobotMap.HIGH_LENGTH_CONE)));

    m_oi.button9.onTrue(new moveTeleToPos(m_teleGrip, m_arm, m_telePid, RobotMap.TELE_MIN_LENGTH+3)
    .andThen(new moveArm(m_arm, m_armPid, RobotMap.MID_CONE_ANGLE))
    .andThen(new moveTeleToPos(m_teleGrip, m_arm, m_telePid, RobotMap.MID_LENGTH_CONE)));

    // //CUBE BUTTONS 8,10
    m_oi.button8.onTrue(new moveTeleToPos(m_teleGrip, m_arm, m_telePid, RobotMap.TELE_MIN_LENGTH+3)
    .andThen(new moveArm(m_arm, m_armPid, RobotMap.HIGH_CUBE_ANGLE))
    .andThen(new moveTeleToPos(m_teleGrip, m_arm, m_telePid, RobotMap.HIGH_LENGTH_CUBE)));

    m_oi.button10.onTrue(new moveTeleToPos(m_teleGrip, m_arm, m_telePid, RobotMap.TELE_MIN_LENGTH+3)
    .andThen(new moveArm(m_arm, m_armPid, RobotMap.MID_CUBE_ANGLE))
    .andThen(new moveTeleToPos(m_teleGrip, m_arm, m_telePid, RobotMap.MID_LENGTH_CUBE)));

    // m_oi.button8.onTrue(new moveArm(m_arm, m_armPid, RobotMap.HIGH_CUBE_ANGLE));
    // m_oi.button10.onTrue(new moveArm(m_arm, m_armPid, RobotMap.MID_CUBE_ANGLE));

    // //low (back and front)
    m_oi.button11.onTrue(new moveTeleToPos(m_teleGrip, m_arm, m_telePid, RobotMap.TELE_MIN_LENGTH+3)
    .andThen(new moveArm(m_arm, m_armPid, RobotMap.LOW_BACK_ANGLE))
    .andThen(new moveTeleToPos(m_teleGrip, m_arm, m_telePid, RobotMap.LOW_LENGTH_BACK)));

    m_oi.button12.onTrue(new moveTeleToPos(m_teleGrip, m_arm, m_telePid, RobotMap.TELE_MIN_LENGTH+3)
    .andThen(new moveArm(m_arm, m_armPid, RobotMap.LOW_FRONT_ANGLE))
    .andThen(new moveTeleToPos(m_teleGrip, m_arm, m_telePid, RobotMap.LOW_LENGTH_FRONT)));

    // m_oi.button11.onTrue(new moveArm(m_arm, m_armPid, RobotMap.LOW_BACK_ANGLE));
    // m_oi.button12.onTrue(new moveArm(m_arm, m_armPid, RobotMap.LOW_FRONT_ANGLE));

    // m_oi.A.onTrue(new ChangeDriverVelocity(driveTrain));
  }
}