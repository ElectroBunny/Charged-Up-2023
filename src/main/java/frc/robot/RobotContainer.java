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
import frc.robot.Commands.moveArm;
import frc.robot.Commands.moveArmManually;
import frc.robot.Commands.moveArmToAngle;
// import frc.robot.Commands.moveArmToAngle;
import frc.robot.Commands.moveTeleManually;
import frc.robot.Commands.moveTeleToPos;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Gripper;
import frc.robot.Subsystems.Telescope;
// import frc.robot.Utilities.MPU6050;


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

  // private MPU6050 mpu;

  CvSink cvSink;
  CvSource outputStream;

  public RobotContainer() {
    driveTrain = new DriveTrain();
    m_gripper = new Gripper();
    m_oi = new OI();
    m_arm = new Arm();
    m_telescope = new Telescope();

    pcmCompressor = new Compressor(PneumaticsModuleType.CTREPCM);
    pcmCompressor.enableDigital();
    // pcmCompressor.enableAnalog(0, 120);

    // mpu = new MPU6050();

    configureButtonBindings();
  }

  public void startCamera(){
    CameraServer.startAutomaticCapture();
    cvSink = CameraServer.getVideo();
    outputStream = CameraServer.putVideo("Blur", 640, 480);
  }


  public void onRobotPeriodic(){}

  public void onAutoInit(double driveTime, double angleSetpoint, double teleSetpoint){
    //Saves the time when the autonomus started.
    this.startTime = Timer.getFPGATimestamp();

    // // m_arm.setSetpointAngle(55);
    // // while (!m_arm.armAtSetPoint()){
    // //   m_arm.move_arm();
    // //   delta_time = Timer.getFPGATimestamp() - startTime;

    // //   if(delta_time >= 15){
    // //     return;
    // //   }
    // // }
    // // m_arm.resist();
    // m_gripper.gripRelease();

    // m_telescope.setSetpointLength(RobotMap.TELE_MIN_LENGTH);
    // while (!this.m_telescope.teleAtSetPoint()){
    //   m_telescope.moveTeleToLength();
    //   delta_time = Timer.getFPGATimestamp() - startTime;

    //   if(delta_time >= 15){
    //     return;
    //   }
    // }
    // m_telescope.stopTele();


    // // send arm to angle
    // m_arm.setSetpointAngle(angleSetpoint);
    // while (!m_arm.armAtSetPoint()){
    //   m_arm.move_arm();
    //   delta_time = Timer.getFPGATimestamp() - startTime;

    //   if(delta_time >= 15){
    //     return;
    //   }
    // }
    // m_arm.resist();


    // // open tele
    // m_telescope.setSetpointLength(teleSetpoint);
    // while (!this.m_telescope.teleAtSetPoint()){
    //   m_telescope.moveTeleToLength();
    //   delta_time = Timer.getFPGATimestamp() - startTime;

    //   if(delta_time >= 15){
    //     return;
    //   }
    // }
    // m_telescope.stopTele();


    // // release gripper
    // m_gripper.gripGrab();


    // // close tele to zero
    // m_telescope.setSetpointLength(RobotMap.TELE_MIN_LENGTH);
    // while (!this.m_telescope.teleAtSetPoint()){
    //   m_telescope.moveTeleToLength();
    //   delta_time = Timer.getFPGATimestamp() - startTime;

    //   if(delta_time >= 15){
    //     return;
    //   }
    // }
    // m_telescope.stopTele();


    // // send arm to low
    // m_arm.setSetpointAngle(RobotMap.LOW_BACK_ANGLE);
    // while (!m_arm.armAtSetPoint()){
    //   m_arm.move_arm();
    //   delta_time = Timer.getFPGATimestamp() - startTime;

    //   if(delta_time >= 15){
    //     return;
    //   }
    // }
    // m_arm.resist();

    this.autoDriveStartTime = Timer.getFPGATimestamp();
    
    while (Timer.getFPGATimestamp() - this.autoDriveStartTime < driveTime){
      driveTrain.ArcadeDrive(0.3, 0); //0.6
      delta_time = Timer.getFPGATimestamp() - startTime;
      
      if(delta_time >= 15){
        return;
      }
    }
    delta_time = 0;
  }

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
    
    driveTrain.setDefaultCommand(new ArcadeDrive(driveTrain));
    // m_oi.A.onTrue(new ChangeDriveVelocity(driveTrain));

    m_oi.button1.onTrue(new Grip(m_gripper));

    m_oi.povbutton1.whileTrue(new moveTeleManually(m_telescope, 1, m_arm));
    m_oi.povbutton2.whileTrue(new moveTeleManually(m_telescope, -1, m_arm));

    m_oi.button2.whileTrue(new moveArmManually(m_arm));


    // 7 - CONE HIGH
    m_oi.button7.onTrue(new moveTeleToPos(m_telescope, RobotMap.TELE_MIN_LENGTH+3)
    .andThen(new moveArm(m_arm, RobotMap.HIGH_CONE_ANGLE)));
    // .andThen(new moveTeleToPos(m_telescope, RobotMap.HIGH_LENGTH_CONE)));
    
    // 9 - CONE MID
    m_oi.button9.onTrue(new moveTeleToPos(m_telescope, RobotMap.TELE_MIN_LENGTH+3)
    .andThen(new moveArm(m_arm, RobotMap.MID_CONE_ANGLE)));
    // .andThen(new moveTeleToPos(m_telescope, RobotMap.MID_LENGTH_CONE)));

    // 8 - CUBE HIGH
    m_oi.button8.onTrue(new moveTeleToPos(m_telescope, RobotMap.TELE_MIN_LENGTH+3)
    .andThen(new moveArm(m_arm, RobotMap.HIGH_CUBE_ANGLE)));
    // .andThen(new moveTeleToPos(m_telescope, RobotMap.HIGH_LENGTH_CUBE)));

    // 10 - CUBE MID
    m_oi.button10.onTrue(new moveTeleToPos(m_telescope, RobotMap.TELE_MIN_LENGTH+3)
    .andThen(new moveArm(m_arm, RobotMap.MID_CUBE_ANGLE)));
    // .andThen(new moveTeleToPos(m_telescope, RobotMap.MID_LENGTH_CUBE)));

    // 11 - LOW BACK (INSIDE THE ROBOT)
    m_oi.button11.onTrue(new moveTeleToPos(m_telescope, RobotMap.TELE_MIN_LENGTH+3)
    .andThen(new moveArm(m_arm, RobotMap.LOW_BACK_ANGLE)));
    // .andThen(new moveTeleToPos(m_telescope, RobotMap.LOW_LENGTH_BACK)));

    // 12 - FEEDER
    m_oi.button12.onTrue(new moveTeleToPos(m_telescope, RobotMap.TELE_MIN_LENGTH+3)
    .andThen(new moveArm(m_arm, RobotMap.LOW_FRONT_ANGLE)));
    // .andThen(new moveTeleToPos(m_telescope, RobotMap.LOW_LENGTH_FRONT)));

    m_oi.button3.onTrue(new moveArmToAngle(m_arm, 90));
  }
}