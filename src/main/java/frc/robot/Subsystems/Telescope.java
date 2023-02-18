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

public class Telescope extends SubsystemBase {
  private WPI_VictorSPX m_teleGrip;
  // Gain contains value of constant in order to open Telescope
  private double Gain;

  private Encoder encoder;
  private PIDCalc encoderPID;
  private double setpointLength;

  public Telescope() {
    this.m_teleGrip = new WPI_VictorSPX(RobotMap.TELESCOPIC_GRIPPER);
    this.m_teleGrip.setNeutralMode(NeutralMode.Coast);

    this.encoder = new Encoder(RobotMap.TELE_ENCODER_CHANNEL_A, RobotMap.TELE_ENCODER_CHANNEL_B, false, EncodingType.k2X);
    this.encoder.setDistancePerPulse(1./2048.);
    this.encoder.reset(); 

    this.encoderPID = new PIDCalc(RobotMap.KP_TELE, RobotMap.KI_TELE, RobotMap.KD_TELE, RobotMap.TOLRENCE_TELE);

  }

  
  // Encoder data functions.

  /*
  returns the distance since last reset (summing)
  */
  public double getDistance(){
    return encoder.getDistance();
  }

  /** Return the length of the arm relative to zero point - where the tele starts.
    *
    * @return the length of the arm as described above.
   */
  public double getLength(){
    return encoder.getDistance() * RobotMap.TELE_SHAFT_PERMITER / RobotMap.TELE_GEAR_RATIO;
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
  /** Gets length and sets its attribute to the setpoint of the PID.
    * @param length the length that will be the setpoint.
   */
  public void setSetpointLength(double length){
    this.setpointLength = length;
  }
  
  /** Sets voltage to the motors using the PID calculations. */
  public void moveTeleToPos(){
    m_teleGrip.set(encoderPID.getOutput(this.getLength(), this.setpointLength));
  }

  /*
   * Function that sets the gain(voltage in percent) to give to the telescope.
   */
  public void setGain(double Gain){
    this.Gain = Gain;
  }

  /** Moves the telescope with a limit.
    *
    * @param currentAngle the current angle of the arm.
    */
  public void moveTeleManually(double currentAngle){
    if((0 < currentAngle && currentAngle < 360) && (this.getLength() < RobotMap.TELE_MAX_LENGTH)){
      m_teleGrip.set(this.Gain);
    }
  }

  public void stopTele(){
    m_teleGrip.stopMotor();
  }

  @Override
  public void periodic() {
  }
}