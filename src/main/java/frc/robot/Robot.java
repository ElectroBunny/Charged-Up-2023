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
  private static final String kLeftAutoCone = "Left Auto Cone";
  private static final String kLeftAutoCube = "Left Auto Cube";
  private static final String kMidAutoCone = "Mid Auto Cone";
  private static final String kMidAutoCube = "Mid Auto Cube";
  private static final String kRightAutoCone = "Right Auto Cone";
  private static final String kRightAutocube = "Right Auto Cube";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final RobotContainer m_robotContainer = new RobotContainer();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    //Code for logs
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Left Auto Cone", kLeftAutoCone);
    m_chooser.addOption("Left Auto Cube", kLeftAutoCube);
    m_chooser.addOption("Mid Auto Cone", kMidAutoCone);
    m_chooser.addOption("Mid Auto Cube", kMidAutoCube);
    m_chooser.addOption("Right Auto Cone", kRightAutoCone);
    m_chooser.addOption("Right Auto Cube", kRightAutocube);

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
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);

    // m_robotContainer.onAutoInit();
    }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // switch (m_autoSelected) {
    //   case kLeftAutoCone:
    //     m_robotContainer.onSimpleAuto(RobotMap.HIGH_CONE_ANGLE);//move_by_color(color, high/middle/low)
    //     break;
    //   case kLeftAutoCube:
    //     m_robotContainer.onSimpleAuto(RobotMap.HIGH_CUBE_ANGLE);
    //     break;
    //   case kMidAutoCone:
    //     m_robotContainer.onAutoMid();
    //     break;

    //   case kMidAutoCube:
    //     m_robotContainer.onAutoMid();
    //     break;

    //   case kRightAutoCone:
    //     m_robotContainer.onSimpleAuto(RobotMap.HIGH_CONE_ANGLE);
    //     break;
    //   case kRightAutocube:
    //     m_robotContainer.onSimpleAuto(RobotMap.HIGH_CUBE_ANGLE);
    //     break;

    //   case kDefaultAuto:
    //   default:
    //     // Put default auto code here
    //     break;
    // }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

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
