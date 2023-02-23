// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase {
  // Definition of motor controllers for the Arm system.
  private WPI_VictorSPX armRightMotor;
  private WPI_VictorSPX armLeftMotor;

  private PIDCalc encoderPID;
  private Encoder encoder;
  private double setpointAngle;

  private double voltPID; 

  public Arm() {
    // Definition of the arm encoder and its constants.
    this.encoder = new Encoder(RobotMap.ARM_ENCODER_CHANNEL_A, RobotMap.ARM_ENCODER_CHANNEL_B, true, EncodingType.k2X);
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

  public void changeToCoast(){
    armRightMotor.setNeutralMode(NeutralMode.Coast);
    armLeftMotor.setNeutralMode(NeutralMode.Coast);
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
    return RobotMap.MIN_ANGLE + (encoder.getDistance() * 360);
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
    this.voltPID = encoderPID.getOutput(this.getAngle(), this.setpointAngle);
    // if (this.voltPID > 0.8){
    //   armRightMotor.set((0 - voltPID)*(0 - voltPID));
    // }
    armRightMotor.set(0 - voltPID);

    SmartDashboard.putNumber("output: ", voltPID);
    // armRightMotor.set(0 - ()));
  }

  /** Moves the arm by dynamic gain given.
   *
   * @param armGain the gain to set the arm motor to(dynamic)
   */
  public void moveArmManually(double armGain){
    armRightMotor.set(armGain);
  }

  /** Holds the arm in its position by giving dynamic gain.
   * 
   * @param teleLength the current length of the telescope, used to calculate the gain in order to hold the arm.
   */
  public void resist(double teleLength){

    if(this.getAngle() <= 178.5){
      armRightMotor.set(-0.2);
    }
    else{
      armRightMotor.set(0.27);
    }
  }

  public void stopArm(){ 
    armRightMotor.stopMotor();
  }

  @Override
  public void periodic() {
  }
 
}