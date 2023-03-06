// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /*
   * Definition of autonomus commands chooser
   */

  private static final String kDefaultAuto = "Default Auto";
  private static final String kCubeMid = "Cube mid";
  private static final String kCubeHigh = "Cube high";
  private static final String kConeMid = "Cone mid";
  private static final String kConeHigh = "Cone High";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final RobotContainer m_robotContainer = new RobotContainer();

  /**
   * This function uns when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_robotContainer.resetEncoders();

    //Code for logs
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    //Camera
    m_robotContainer.startCamera();

    //Autonomus choice
    m_chooser.setDefaultOption("Please choose option", kDefaultAuto);
    m_chooser.addOption("Cube mid", kCubeMid);
    m_chooser.addOption("Cube High", kCubeHigh);
    m_chooser.addOption("Cone mid", kConeMid);
    m_chooser.addOption("Cone high", kConeHigh);


    SmartDashboard.putData("Auto choices", m_chooser);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.onRobotPeriodic();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();

    switch(m_autoSelected)
    {
      case kCubeMid:
        m_robotContainer.onAutoInit(5, RobotMap.MID_CUBE_ANGLE, RobotMap.MID_LENGTH_CUBE);
        break;
      case kCubeHigh:
        m_robotContainer.onAutoInit(5, RobotMap.HIGH_CUBE_ANGLE, RobotMap.HIGH_LENGTH_CUBE);
        break;
      case kConeMid:
        m_robotContainer.onAutoInit(5, RobotMap.MID_CONE_ANGLE, RobotMap.MID_LENGTH_CONE);
        break;
      case kConeHigh:
        m_robotContainer.onAutoInit(5, RobotMap.HIGH_CONE_ANGLE, RobotMap.HIGH_LENGTH_CONE);
        break;
      default:
        break;
      }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_robotContainer.onTeleopInit();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    m_robotContainer.onDisabledInit();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
