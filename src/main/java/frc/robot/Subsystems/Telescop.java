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

public class Telescop extends SubsystemBase {
  private WPI_VictorSPX m_teleGrip;

  private Encoder encoder;
  private PIDCalc encoderPID;
  private double setpointLength;

  public Telescop() {
    this.m_teleGrip = new WPI_VictorSPX(RobotMap.TELESCOPIC_GRIPPER);
    this.m_teleGrip.setNeutralMode(NeutralMode.Coast);

    encoder = new Encoder(RobotMap.TELE_ENCODER_CHANNEL_A, RobotMap.TELE_ENCODER_CHANNEL_B, false, EncodingType.k2X);
    encoder.setDistancePerPulse(1./2048.);
    encoder.reset(); 

    encoderPID = new PIDCalc(RobotMap.KP_TELE, RobotMap.KI_TELE, RobotMap.KD_TELE, RobotMap.TOLRENCE_TELE);

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
    return encoder.getDistance() * RobotMap.TELE_GEAR_PERMITER / RobotMap.TELE_GEAR_RATIO;
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

  @Override
  public void periodic() {
  }
}