// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase {
  // Definition of motor controllers for the Arm system.
  private WPI_VictorSPX armRightMotor;
  private WPI_VictorSPX armLeftMotor;

  private PIDCalc encoderPID;
  private Encoder encoder;
  private double setpointAngle;
  

  public Arm() {
    // Definition of the arm encoder and its constants.
    this.encoder = new Encoder(RobotMap.ARM_ENCODER_CHANNEL_A, RobotMap.ARM_ENCODER_CHANNEL_B, false, EncodingType.k2X);
    this.encoder.setDistancePerPulse(1./2048.);
    this.encoder.reset();

    // Definiton of the PID object by its constants.
    this.encoderPID = new PIDCalc(RobotMap.KP_ARM, RobotMap.KI_ARM, RobotMap.KD_ARM, RobotMap.TOLRENCE_ARM);

    // Initializes arm motors.
    this.armRightMotor = new WPI_VictorSPX(RobotMap.ARM_RIGHT_MOTOR);
    this.armLeftMotor = new WPI_VictorSPX(RobotMap.ARM_LEFT_MOTOR);

    this.armLeftMotor.setNeutralMode(NeutralMode.Coast);
    this.armRightMotor.setNeutralMode(NeutralMode.Coast);

    this.armLeftMotor.setInverted(true);
    this.armRightMotor.setInverted(false);

    this.armLeftMotor.follow(armRightMotor);
  }

  // Encoder data functions.

  /*
  returns the distance since last reset (summing)
  */
  public double getDistance(){
    return encoder.getDistance();
  }

  /** Return the angle of the arm relative to zero point - down to the floor, the positive direction is the front of the robot direction.
    *
    * @return the angle of the arm as described above.
   */
  public double getAngle(){
    return RobotMap.MIN_ANGLE + (encoder.getDistance() * 360 / RobotMap.ARM_GEAR_RATIO);
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


  // PID functions
  /** Gets angle and sets its attribute to the setpoint of the PID.
    * @param angle the angle that will be the setpoint.
   */
  public void setSetpointAngle(double angle){
    this.setpointAngle = angle;
  }
  
  /** Sets voltage to the motors using the PID calculations. */
  public void moveArmToAngle(){
    armRightMotor.set(encoderPID.getOutput(this.getAngle(), this.setpointAngle));
  }

  @Override
  public void periodic() {
  }
 
}
