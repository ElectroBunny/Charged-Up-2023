// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/**
Decleration of talon (controlling voltage)
 */
public class DriveTrain extends SubsystemBase {
  
  private Talon m_rightMaster = new Talon(RobotMap.DRIVE_RIGHT_MASTER);
  private Talon m_rightSlave = new Talon(RobotMap.DRIVE_RIGHT_SLAVE);
  private Talon m_leftMaster = new Talon(RobotMap.DRIVE_LEFT_MASTER);
  private Talon m_leftSlave = new Talon(RobotMap.DRIVE_LEFT_SLAVE);
  private DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMaster, m_rightMaster);

  public DriveTrain() {

    /*
    * Decleration of opposite directions.
    */
    m_rightMaster.setInverted(true);
    m_rightSlave.setInverted(true);
    m_leftMaster.setInverted(false);
    m_leftSlave.setInverted(false);

    

  }

  public void ArcadeDrive(double forward, double turn){
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
