// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;


public class Arm extends SubsystemBase {
  // Definition of motor controllers for the Arm system.
  // 1234
  private WPI_VictorSPX armRightMotor;
  private WPI_VictorSPX armLeftMotor;

  private Encoder encoder;
  private PIDCalc armPID;
  private double setpointAngle;
  private double voltPID; 

  private double currentAngle;

  private static Arm instance = null;

  public Arm() {
    // Definition of the arm encoder and its constants.
    this.encoder = new Encoder(RobotMap.ARM_ENCODER_CHANNEL_A, RobotMap.ARM_ENCODER_CHANNEL_B, true, EncodingType.k2X);
    this.encoder.setDistancePerPulse(1./2048.);
    this.encoder.reset();

    this.armPID = new PIDCalc(RobotMap.KP_ARM, RobotMap.KI_ARM, RobotMap.KD_ARM, RobotMap.TOLRENCE_ARM);

    // Initializes arm motors.
    this.armRightMotor = new WPI_VictorSPX(RobotMap.ARM_RIGHT_MOTOR);
    this.armLeftMotor = new WPI_VictorSPX(RobotMap.ARM_LEFT_MOTOR);

    this.armLeftMotor.setNeutralMode(NeutralMode.Brake);
    this.armRightMotor.setNeutralMode(NeutralMode.Brake);

    this.armLeftMotor.setInverted(true);
    this.armRightMotor.setInverted(false);

    this.armLeftMotor.follow(armRightMotor);


    this.currentAngle = RobotMap.MIN_ANGLE;
  }

  public void changeToCoast(){
    armRightMotor.setNeutralMode(NeutralMode.Coast);
    armLeftMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void changeToBrake() {
    armRightMotor.setNeutralMode(NeutralMode.Brake);
    armLeftMotor.setNeutralMode(NeutralMode.Brake);
  }

  /* ----- Encoder data functions ----- */

  /** Resets the encoder measurement that counted during the last run. */
  public void resetEncoder(){
    this.encoder.reset();
  }

  /** Return the angle of the arm relative to zero point - down to the floor, the positive direction is the front of the robot direction.
    *
    * @return the angle of the arm as described above.
   */
  public double getAngle(){
    return RobotMap.MIN_ANGLE + (encoder.getDistance() * 360);
  }
  

  /* ----- PID functions ----- */

  /** Gets angle and sets its attribute to the setpoint of the PID.
    * @param angle the angle that will be the setpoint.
   */
  public void setSetpointAngle(double angle){
    this.setpointAngle = angle;
    this.armPID.setSetpoint(angle);
  }

  /** Used to check if the arm has reached its setpoint angle.
    *
    * @return true if the arm has reached its setpoint, otherwise - false.
   */
  public boolean armAtSetPoint(){
    this.currentAngle = this.getAngle();
    return this.armPID.atSetPoint(this.currentAngle);
  }
  
  /** Sets voltage to the motors using the PID calculations.*/
  public void moveArmToAngle(){
    this.voltPID = this.armPID.getOutput(this.getAngle());
    this.armRightMotor.set(MathUtil.clamp(0 - (this.voltPID + 0.25 * Math.sin(Math.toRadians(this.getAngle()))), -1, 1));
  }

  /** Use to change the setpoint angle manualy.
   * @param gain how many and witch direction change the setpoint.
   */
  public void moveSetPoint(double gain){
    this.currentAngle = this.getAngle();
    if (gain > 0){
      if (this.currentAngle <= RobotMap.MAX_ANGLE){
        this.setpointAngle += gain * 5;
      }
      else {
        this.resist();
      }
    }
    else if (gain < 0){
      if (this.currentAngle >= RobotMap.MIN_ANGLE){
        this.setpointAngle -= gain * 5;
      }
      else {
        this.resist();
      }
    }
    else {
      this.resist();
    }
    this.setSetpointAngle(this.setpointAngle);
  }

  /** Used to move the arm from it angle to other angle automatically, considering the velocity needed in each direction. */
  public void move_arm(){
    this.currentAngle = this.getAngle();
    if (this.currentAngle < 180 && this.setpointAngle < 180){
      if (this.currentAngle < this.setpointAngle){
        // more volt (-)
        armRightMotor.set(0 - RobotMap.ARM_RAISE_VOLT);
      }
      else {
        // less volt (+)
        if (this.currentAngle > 160){
          armRightMotor.set(RobotMap.ARM_LOWER_VOLT + 0.22);
        }
        else {
          armRightMotor.set(RobotMap.ARM_LOWER_VOLT);
        }
      }
    }
    else if (this.currentAngle > 180 && this.setpointAngle > 180){
      if (this.currentAngle < this.setpointAngle){
        // less volt (+)
        if (this.currentAngle < 200){
          armRightMotor.set(0 - (RobotMap.ARM_LOWER_VOLT + 0.22));
        }
        else {
          armRightMotor.set(RobotMap.ARM_LOWER_VOLT);
        }
      }
      else {
        // more volt (-)
        armRightMotor.set(RobotMap.ARM_RAISE_VOLT);
      }
    }
    else if (this.currentAngle <= 180 && this.setpointAngle > 180){
      // more volt (-)
      armRightMotor.set(0 - RobotMap.ARM_RAISE_VOLT);
    }
    else if (this.currentAngle >= 180 && this.setpointAngle < 180){
      // more volt (+)
      armRightMotor.set(RobotMap.ARM_RAISE_VOLT);
    }
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
  public void resist(){
    this.currentAngle = this.getAngle();

    SmartDashboard.putNumber("Resist Angle", this.currentAngle);
    // armRightMotor.set(Math.sin(Math.toRadians(this.currentAngle)) * RobotMap.RESIST_VOLT);

    if (this.getAngle() <= 165){
      armRightMotor.set(-0.25);
    }
    else if (this.getAngle() <= 180){
      armRightMotor.set(0);
    }
    else if (this.getAngle() <= 205){
      armRightMotor.set(0);
    }
    else{
      armRightMotor.set(0.25);
    }
  }

  /** Used to stop the arm motors. */
  public void stopArm(){ 
    armRightMotor.stopMotor();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Angle", this.getAngle());
  }

  public static Arm getInstance() {
    if (instance == null) {
      instance = new Arm();
    }
    return instance;
  }
}