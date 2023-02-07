// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

// The integral gain term will never add or subtract more than 0.5 from
// the total loop output
//pid.setIntegratorRange(-0.5, 0.5);

// takes a value and put him in range of (-0.5, 0.5) by propotion. This value will be used to clamp the output value of the pid and use it in the motors.
//MathUtil.clamp(pid.calculate(encoder.getDistance(), setpoint), -0.5, 0.5);

public class ArmPID extends SubsystemBase {
  /*
   * The constructor of the pid controller, which controls the arm movement.
   */
  PIDController pid;
  Encoder right_encoder;
  double v;

  public ArmPID() {
    pid = new PIDController(RobotMap.KP_ARM, RobotMap.KI_ARM, RobotMap.KD_ARM);
    right_encoder = new Encoder(RobotMap.RIGHT_ENCODER_CHANNEL_A, RobotMap.RIGHT_ENCODER_CHANNEL_B, false, EncodingType.k2X);
    right_encoder.setDistancePerPulse(1./2048.);
    right_encoder.reset();
  }

  /*
   * Encoders functions.
   */
  public double getDistanceRight(){
    return right_encoder.getDistance();
  }

  public double getAngleRight(){
    return right_encoder.getDistance() * 360;
  }

  public double getRateRight(){
    return right_encoder.getRate();
  }

  public boolean getDirectRight(){
    return right_encoder.getDirection();
  }

  /*
   * PID calculation functions.
   */
  public double changeSideToGripper(boolean isRight)
  {
    if (isRight){
      return setToAngle(RobotMap.MIN_ANGLE);
    }
    else {
      return setToAngle(RobotMap.MAX_ANGLE);
    }
    isRight != isRight;
  }

  public double setToAngle(double setpoint)
  {
    pid.setTolerance(RobotMap.TOLRENCE_ARM);
    v = MathUtil.clamp(pid.calculate(getAngleRight(), setpoint), -1, 1);
    return v;
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
