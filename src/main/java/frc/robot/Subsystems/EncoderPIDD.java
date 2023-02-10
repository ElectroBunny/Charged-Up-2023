// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class EncoderPIDD extends SubsystemBase{

  /*
  decleration of encoder veriables
  */
  PIDController pid;
  Encoder encoder;
  double v;

  /*
  constructor + reset of pid values from previous runs
  */
  public EncoderPIDD(double kP, double kI, double kD, double tolerance, Encoder encoder) {
    pid.reset();
    pid = new PIDController(kP, kI, kD);
    pid.setTolerance(tolerance);

    this.encoder = encoder;
  }

  /*
  returns the distance since last reset (summing)
  */
  public double getDistance(){
    return encoder.getDistance();
  }

  /*
  returns the angle since last reset (distance*360/50)
  */
  public double getAngle(){
    return encoder.getDistance() * 7.2;
  }

  /*
  rate of change per second since last reset
  */
  public double getRate(){
    return encoder.getRate();
  }

  public boolean getDirection(){
    return encoder.getDirection();
  }

  public double setToDistance(double setpoint){
    pid.setTolerance(RobotMap.TOLRENCE_ARM);
    v = MathUtil.clamp(pid.calculate(getDistance(), setpoint), -1, 1);
    return v;
  }
  
  public double setToAngle(double setpoint){
    pid.setTolerance(RobotMap.TOLRENCE_ARM);
    v = MathUtil.clamp(pid.calculate(getAngle(), setpoint), -1, 1);
    return v;
  }

  public boolean atSetPoint(){
    return pid.atSetpoint();
  }
  
  public void resetPID(){
    pid.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
