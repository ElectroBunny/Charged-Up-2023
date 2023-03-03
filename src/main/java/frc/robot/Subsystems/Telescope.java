// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private double currentLength;

  public Telescope() {
    this.m_teleGrip = new WPI_VictorSPX(RobotMap.TELESCOPIC_GRIPPER);
    this.m_teleGrip.setNeutralMode(NeutralMode.Coast);

    this.encoder = new Encoder(RobotMap.TELE_ENCODER_CHANNEL_A, RobotMap.TELE_ENCODER_CHANNEL_B, true, EncodingType.k2X);
    this.encoder.setDistancePerPulse(1./2048.);
    this.encoder.reset(); 

    this.encoderPID = new PIDCalc(RobotMap.KP_TELE, RobotMap.KI_TELE, RobotMap.KD_TELE, RobotMap.TOLRENCE_TELE);
    this.currentLength = RobotMap.TELE_MIN_LENGTH;
  }

  
  // Encoder data functions.

  /*
  returns the distance since last reset (summing)
  */
  public double getDistance(){
    return encoder.getDistance();
  }

  public void resetEn(){
    this.encoder.reset();
  }

  /** Return the length of the arm relative to zero point - where the tele starts.
    *
    * @return the length of the arm as described above.
   */
  public double getLength(){
    return encoder.getDistance() * RobotMap.TELE_SHAFT_PERMITER;
  }

  /*
  rate of change per second since last reset
  */
  // public double getRate(){
  //   return encoder.getRate();
  // }

  // public boolean getDirection(){
  //   return encoder.getDirection();
  // }


  // PID functions
  /** Gets length and sets its attribute to the setpoint of the PID.
    * @param length the length that will be the setpoint.
   */
  public void setSetpointLength(double length){
    this.setpointLength = length;
  }
  
  /** Sets voltage to the motors using the PID calculations. */
  public void moveTeleToLength(){
    if (this.setpointLength - (this.getLength() + RobotMap.TELE_MIN_LENGTH) > 0){
      m_teleGrip.set(0.5);
    }
    else {
      m_teleGrip.set(-0.5);
    }
    // m_teleGrip.set(encoderPID.getOutput(this.getLength(), this.setpointLength));
  }

  /*
   * Function that sets the gain(voltage in percent) to give to the telescope.
   */
  public void setGain(double Gain){
    this.Gain = Gain;
  }

  /** Moves the .settelescope with a limit.
    *
    * @param currentAngle the current angle of the arm.
    */
  public void moveTeleManually(double gain, double currentAngle){

    if(currentAngle > RobotMap.MIN_ANGLE && currentAngle < 58){
      return;
    }
    this.currentLength = RobotMap.TELE_MIN_LENGTH + this.getLength();

    gain *= 0.8;
    m_teleGrip.set(gain);
    // // SmartDashboard.putNumber("TELE realative length", this.getLength());
    // // SmartDashboard.putNumber("TELE current angle", currentAngle);
    // // SmartDashboard.putNumber("TELE full length", this.currentLength);
    // if (gain > 0){ // If trying to open
    //   if (this.currentLength < RobotMap.TELE_MAX_LENGTH) { // Is possible to open?
    //     if (RobotMap.VERTICAL_FIRST_ANGLE <=  currentAngle && currentAngle <= RobotMap.VERTICAL_SECOND_ANGLE){ // If angle in vertical laws
    //       if (this.currentLength < RobotMap.MAX_VERTICAL_LENGTH){ // If length < law
    //         m_teleGrip.set(gain); // Set power
    //       }
    //       else { // Length >= law
    //         m_teleGrip.set(0); // Clear power
    //       }
    //     }
    //     else if (RobotMap.HORIZONTAL_FIRST_ANGLE <=  currentAngle && currentAngle <= RobotMap.HORIZONTAL_SECOND_ANGLE){ // If angle in horizontal laws
    //       SmartDashboard.putNumber("horizontagl length", this.currentLength * Math.cos(270-currentAngle));
    //       SmartDashboard.putNumber("curr length", this.currentLength);
    //       SmartDashboard.putNumber("angle (cos)", 270 - currentAngle);
    //       SmartDashboard.putNumber("cos", Math.cos(270 - currentAngle));
    //       if ((this.currentLength * Math.cos(currentAngle - 270)) < RobotMap.MAX_HORIZONTAL_LENGTH){ // If horizontal length < law
    //         m_teleGrip.set(gain); // Set power
    //       }
    //       else { // Horizontal length >= law
    //         m_teleGrip.set(0); // Clear power
    //       }
    //     }
    //     else { // Angle not in law.
    //       m_teleGrip.set(gain); // Set power
    //     }
    //   }
    //   else { // Impossible to open
    //     m_teleGrip.set(0); // Clear power
    //   }
    // }
    // else if (gain < 0) { // Trying to close
    //   if (this.currentLength > RobotMap.TELE_MIN_LENGTH){ // Is possible to close?
    //     m_teleGrip.set(gain); // Set power
    //   }
    //   else { // Impossible to close
    //     m_teleGrip.set(0); // Clear power
    //   }
    //   // m_teleGrip.set(gain);
    // }
    // else { // Not trying to open or close
    //   m_teleGrip.set(0); // Clear power
    // }



    // m_teleGrip.set(0.5 * gain);
    // SmartDashboard.putNumber("Gain", gain);
    // SmartDashboard.putNumber("Relative length", this.getLength());
    // SmartDashboard.putNumber("Full length", this.getLength() + RobotMap.TELE_MIN_LENGTH);
  }

  public void stopTele(){
    m_teleGrip.stopMotor();
  }

  @Override
  public void periodic() {
  }
}