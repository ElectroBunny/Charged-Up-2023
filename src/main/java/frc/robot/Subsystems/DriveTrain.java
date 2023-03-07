// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriveTrain extends SubsystemBase {
  
  private WPI_TalonSRX m_rightMaster;
  private WPI_TalonSRX m_rightFollower;
  private WPI_TalonSRX m_leftMaster;
  private WPI_TalonSRX m_leftFollower;

  private DifferentialDrive m_diffDrive;

  private double driveMul;
  private boolean isFast;

  private static DriveTrain instance = null;


  // private CommandXboxController controller;

  public DriveTrain() {

    this.m_rightMaster = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_MASTER);
    this.m_rightFollower = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_SLAVE);
    this.m_leftMaster = new WPI_TalonSRX(RobotMap.DRIVE_LEFT_MASTER);
    this.m_leftFollower = new WPI_TalonSRX(RobotMap.DRIVE_LEFT_SLAVE);

    // Setting the config of the motors to factory default.
    m_rightMaster.configFactoryDefault();
    m_rightFollower.configFactoryDefault();
    m_leftMaster.configFactoryDefault();
    m_leftFollower.configFactoryDefault();

    // Decleration of opposite directions.
    m_rightMaster.setInverted(false);
    m_rightFollower.setInverted(false);
    m_leftMaster.setInverted(true);
    m_leftFollower.setInverted(true);

    // Setting the neautral mode of the motors to coast.
    m_rightMaster.setNeutralMode(NeutralMode.Brake);
    m_rightFollower.setNeutralMode(NeutralMode.Brake);
    m_leftMaster.setNeutralMode(NeutralMode.Brake);
    m_leftFollower.setNeutralMode(NeutralMode.Brake);

    // Joining masters and followers motor controllers.
    m_rightFollower.follow(m_rightMaster);
    m_leftFollower.follow(m_leftMaster);

    m_diffDrive = new DifferentialDrive(m_leftMaster, m_rightMaster);

    // Driving velocity variables.
    this.driveMul = 1;
    this.isFast = true;

    // this.controller = new CommandXboxController(4);
  }

  /** Function that moves the robot.
    * @param forward power in precentage to move the robot forward (or backward).
    * @param turn power in precentage to move the robot right or left.
    */
  public void ArcadeDrive(double forward, double turn){
    // Deadzone
    if(Math.abs(forward) < 0.2)
      forward = 0.0;

    if(Math.abs(turn) < 0.2)
      turn = 0.0;
    
    m_diffDrive.arcadeDrive(forward, turn);
  }

  /** Used to toggle between the velocity modes of the robot - slow/fast*/
  public void changeVel(){
    if(this.isFast){
      this.driveMul = 0.5;
    }
    else{
      this.driveMul = 1;
    }
    this.isFast = !this.isFast;
  }

  public void StopMotors(){
    m_rightMaster.stopMotor();
    m_leftMaster.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Velocity", driveMul * 100);

    // double leftY = controller.getLeftY();
    // double rightX = controller.getRightX();

    // m_rightMaster.set(leftY - rightX);
    // m_leftMaster.set(leftY + rightX);
  }

  public static DriveTrain getInstance() {
    if (instance == null) {
      instance = new DriveTrain();
    }
    return instance;
  }
  
}
