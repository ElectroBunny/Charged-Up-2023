// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.OI;
import frc.robot.RobotMap;

/**
Decleration of talon (controlling voltage)
 */
public class DriveTrain extends SubsystemBase {
  
  private WPI_TalonSRX m_rightMaster = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_MASTER);
  private WPI_TalonSRX m_rightFollower = new WPI_TalonSRX(RobotMap.DRIVE_RIGHT_SLAVE);
  private WPI_TalonSRX m_leftMaster = new WPI_TalonSRX(RobotMap.DRIVE_LEFT_MASTER);
  private WPI_TalonSRX m_leftFollower = new WPI_TalonSRX(RobotMap.DRIVE_LEFT_SLAVE);
  private DifferentialDrive m_diffDrive;

  private double driveMul = 1;
  private boolean isFast = true;

  private CommandXboxController controller = new CommandXboxController(4);

  public DriveTrain() {

    /*
     * Setting the config of the motors to factory default.
     */
    m_rightMaster.configFactoryDefault();
    m_rightFollower.configFactoryDefault();
    m_leftMaster.configFactoryDefault();
    m_leftFollower.configFactoryDefault();

    /*
    * Decleration of opposite directions.
    */
    m_rightMaster.setInverted(false);
    m_rightFollower.setInverted(false);
    m_leftMaster.setInverted(true);
    m_leftFollower.setInverted(true);

    /*
     * Setting the neautral mode of the motors to coast.
     */
    m_rightMaster.setNeutralMode(NeutralMode.Brake);
    m_rightFollower.setNeutralMode(NeutralMode.Brake);
    m_leftMaster.setNeutralMode(NeutralMode.Brake);
    m_leftFollower.setNeutralMode(NeutralMode.Brake);

    /*
     * Joining masters and followers motor controllers.
     */
    m_rightFollower.follow(m_rightMaster);
    m_leftFollower.follow(m_leftMaster);

    m_diffDrive = new DifferentialDrive(m_leftMaster, m_rightMaster);
  }

  /*
   * Function that connects between the joystick values and the drive train.
   * Used in the teleop mode.
   */
  public void ArcadeDrive(double forward, double turn){
    //Deadzone
    if(Math.abs(forward) < 0.2)
      forward = 0.0;

    if(Math.abs(turn) < 0.2)
      turn = 0.0;
    
    m_diffDrive.arcadeDrive(forward, turn);
  }

  public void runRightSide(double speed) {
    m_rightMaster.set(speed);
  }

  public void runLeftSide(double speed) {
    m_leftMaster.set(speed);
  }

  

  // public void changeVel(){
  //   if(this.isFast){
  //     this.driveMul = 0.5;
  //   }
  //   else{
  //     this.driveMul = 1;
  //   }
  //   this.isFast = !this.isFast;
  // }

  public void StopMotors(){
    m_rightMaster.stopMotor();
    m_leftMaster.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Velocity", driveMul * 100);

    double leftY = controller.getLeftY();
    double rightX = controller.getRightX();

    // m_rightMaster.set(leftY - rightX);
    // m_leftMaster.set(leftY + rightX);
  }
  
}
