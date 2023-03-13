// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Commands.ArcadeDrive;
// import frc.robot.Commands.ChangeDriveVelocity;
import frc.robot.Commands.Grip;
import frc.robot.Commands.moveArmManually;
// import frc.robot.Commands.moveArmToAngle;
import frc.robot.Commands.moveTeleManually;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Gripper;
import frc.robot.Subsystems.Telescope;


public class RobotContainer {

  private final DriveTrain driveTrain;
  private final OI m_oi;
  private final Telescope m_telescope;
  private final Arm m_arm;
  private final Gripper m_gripper;

  private double startTime;
  private double delta_time = 0;
  private double autoDriveStartTime;

  private Compressor pcmCompressor;


  CvSink cvSink;
  CvSource outputStream;

  public RobotContainer() {
    driveTrain = DriveTrain.getInstance();
    m_gripper = Gripper.getInstance();
    m_arm = Arm.getInstance();
    m_telescope = Telescope.getInstance();
    m_oi = new OI();

    pcmCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
    pcmCompressor.enableDigital();
    // pcmCompressor.enableAnalog(0, 120);
    configureButtonBindings();
  }

  public void startCamera(){
    CameraServer.startAutomaticCapture();
    cvSink = CameraServer.getVideo();
    outputStream = CameraServer.putVideo("Blur", 640, 480);
  }


  public void onRobotPeriodic(){}

  public void onAutoInit(double driveTime, double angleSetpoint, double teleSetpoint){
    m_arm.changeToBrake();

    //Saves the time when the autonomus started.
    this.startTime = Timer.getFPGATimestamp();

    // grabs the game piece which is inside the gripper
    m_gripper.gripGrab();

    // closes the tele before movign the arm
    m_telescope.setSetpointLength(RobotMap.TELE_MIN_LENGTH);
    while (!this.m_telescope.teleAtSetPoint()){
      m_telescope.moveTeleToLength();
      delta_time = Timer.getFPGATimestamp() - startTime;

      if(delta_time >= 15){
        return;
      }
    }
    m_telescope.stopTele();


    // sends arm to angle
    m_arm.setSetpointAngle(angleSetpoint);
    while (!m_arm.armAtSetPoint()){
      m_arm.move_arm();
      delta_time = Timer.getFPGATimestamp() - startTime;

      if(delta_time >= 15){
        return;
      }
    }
    m_arm.resist();


    // opens tele
    m_telescope.setSetpointLength(teleSetpoint);
    while (!this.m_telescope.teleAtSetPoint()){
      m_telescope.moveTeleToLength();
      delta_time = Timer.getFPGATimestamp() - startTime;

      if(delta_time >= 15){
        return;
      }
    }
    m_telescope.stopTele();


    // releases gripper
    m_gripper.gripRelease();

    // moves the arm a little bit higher then the grid
    m_arm.setSetpointAngle(angleSetpoint - 10);
    while (!m_arm.armAtSetPoint()){
      m_arm.move_arm();
      delta_time = Timer.getFPGATimestamp() - startTime;

      if(delta_time >= 15){
        return;
      }
    }
    m_arm.resist();

    // close tele to zero
    m_telescope.setSetpointLength(RobotMap.TELE_MIN_LENGTH);
    while (!this.m_telescope.teleAtSetPoint()){
      m_telescope.moveTeleToLength();
      delta_time = Timer.getFPGATimestamp() - startTime;

      if(delta_time >= 15){
        return;
      }
    }
    m_telescope.stopTele();


    // sends arm to low
    m_arm.setSetpointAngle(RobotMap.LOW_BACK_ANGLE);
    while (!m_arm.armAtSetPoint()){
      m_arm.move_arm();
      delta_time = Timer.getFPGATimestamp() - startTime;

      if(delta_time >= 15){
        return;
      }
    }
    m_arm.resist();

    // drives out of the community
    this.autoDriveStartTime = Timer.getFPGATimestamp();
    while (Timer.getFPGATimestamp() - this.autoDriveStartTime < driveTime){
      driveTrain.ArcadeDrive(0.8, 0);
      delta_time = Timer.getFPGATimestamp() - startTime;
      
      if(delta_time >= 15){ 
        return;
      }
    }
    delta_time = 0;
  }

  // drivers's control without Interfere 
  public void onTeleopInit(){
    driveTrain.ArcadeDrive(0, 0);
    m_arm.changeToBrake();
  }

  public void resetEncoders(){
    m_arm.resetEncoder();
    m_telescope.resetEncoder();
  }

  public void onTeleopPeriodic(){
  }

  public void onDisabledInit(){
    m_arm.changeToCoast();
  }


  private void configureButtonBindings() {
    /* ----- Linking between the buttons, that defined in oi.java, to commands. ----- */
    
    driveTrain.setDefaultCommand(new ArcadeDrive());
    // m_oi.A.onTrue(new ChangeDriveVelocity(driveTrain));

    m_oi.button1.onTrue(new Grip());

    m_oi.povbutton1.whileTrue(new moveTeleManually(0.8));
    m_oi.povbutton2.whileTrue(new moveTeleManually(-0.8));

    m_oi.button2.whileTrue(new moveArmManually());


    // // 7 - CONE HIGH
    // m_oi.button7.onTrue(new moveTeleToPos(RobotMap.TELE_MIN_LENGTH+3)
    // .andThen(new moveArm(RobotMap.HIGH_CONE_ANGLE)));
    // // .andThen(new moveTeleToPos(RobotMap.HIGH_LENGTH_CONE)));
    
    // // 9 - CONE MID
    // m_oi.button9.onTrue(new moveTeleToPos(RobotMap.TELE_MIN_LENGTH+3)
    // .andThen(new moveArm(RobotMap.MID_CONE_ANGLE)));
    // // .andThen(new moveTeleToPos(RobotMap.MID_LENGTH_CONE)));

    // // 8 - CUBE HIGH
    // m_oi.button8.onTrue(new moveTeleToPos(RobotMap.TELE_MIN_LENGTH+3)
    // .andThen(new moveArm(RobotMap.HIGH_CUBE_ANGLE)));
    // // .andThen(new moveTeleToPos(RobotMap.HIGH_LENGTH_CUBE)));

    // // 10 - CUBE MID
    // m_oi.button10.onTrue(new moveTeleToPos(RobotMap.TELE_MIN_LENGTH+3)
    // .andThen(new moveArm(RobotMap.MID_CUBE_ANGLE)));
    // // .andThen(new moveTeleToPos(RobotMap.MID_LENGTH_CUBE)));

    // // 11 - LOW BACK (INSIDE THE ROBOT)
    // m_oi.button11.onTrue(new moveTeleToPos(RobotMap.TELE_MIN_LENGTH+3)
    // .andThen(new moveArm(RobotMap.LOW_BACK_ANGLE)));
    // // .andThen(new moveTeleToPos(RobotMap.LOW_LENGTH_BACK)));

    // // 12 - FEEDER
    // m_oi.button12.onTrue(new moveTeleToPos(RobotMap.TELE_MIN_LENGTH+3)
    // .andThen(new moveArm(RobotMap.LOW_FRONT_ANGLE)));
    // // .andThen(new moveTeleToPos(RobotMap.LOW_LENGTH_FRONT)));

  }
}