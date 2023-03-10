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
import frc.robot.RobotMap;

public class Telescope extends SubsystemBase {
  private WPI_VictorSPX m_teleGrip;

  private Encoder encoder;
  private PIDCalc telePID;
  private double setpointLength;

  private double currentLength;

  public Telescope() {
    // Definition of the tele encoder and its constants.
    this.encoder = new Encoder(RobotMap.TELE_ENCODER_CHANNEL_A, RobotMap.TELE_ENCODER_CHANNEL_B, true, EncodingType.k2X);
    this.encoder.setDistancePerPulse(1./2048.);
    this.encoder.reset();

    this.telePID = new PIDCalc(RobotMap.KP_TELE, RobotMap.KI_TELE, RobotMap.KD_TELE, RobotMap.TOLRENCE_TELE);

    // Initializes tele motors.
    this.m_teleGrip = new WPI_VictorSPX(RobotMap.TELESCOPIC_GRIPPER);
    this.m_teleGrip.setNeutralMode(NeutralMode.Brake);


    this.currentLength = RobotMap.TELE_MIN_LENGTH;
  }

  
  /* ----- Encoder data functions. ----- */

  /** Resets the encoder measurement that counted during the last run. */
  public void resetEncoder(){
    this.encoder.reset();
  }

  /** Return the length of the arm relative to zero point - where the tele starts.
    *
    * @return the length of the arm as described above.
   */
  public double getLength(){
    return encoder.getDistance() * RobotMap.TELE_SHAFT_PERMITER;
  }


  /* ----- PID functions ----- */

  /** Gets length and sets its attribute to the setpoint of the PID.
    * @param length the length that will be the setpoint.
   */
  public void setSetpointLength(double length){
    this.setpointLength = length;
    this.telePID.setSetpoint(length);
  }

  /** Used to check if the tele reached its setpoint length.
   * 
   * @return true if the tele reached its setpoint, otherwise - false.
   */
  public boolean teleAtSetPoint(){
    this.currentLength = RobotMap.TELE_MIN_LENGTH + this.getLength();
    return this.telePID.atSetPoint(this.currentLength);
  }
  
  /** Sets voltage to the motors using the PID calculations. */
  public void moveTeleToLength(){
    if (this.setpointLength - (this.getLength() + RobotMap.TELE_MIN_LENGTH) > 0){
      m_teleGrip.set(0.5);
    }
    else {
      m_teleGrip.set(-0.5);
    }

    SmartDashboard.putNumber("Tele", this.getLength() + RobotMap.TELE_MIN_LENGTH);
  }

  /** Moves the .settelescope with a limit.
    *
    * @param currentAngle the current angle of the arm.
  //   */
  public void moveTeleManually(double gain, double currentAngle){

    if(currentAngle > RobotMap.MIN_ANGLE && currentAngle < 58){
      return;
    }
    this.currentLength = RobotMap.TELE_MIN_LENGTH + this.getLength();

    gain *= 0.8;
    m_teleGrip.set(gain);
    
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
    //       if ((this.currentLength * Math.cos(Math.toRadians(currentAngle - 270))) < RobotMap.MAX_HORIZONTAL_LENGTH){ // If horizontal length < law
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
  }
    //m_teleGrip.set(0.5 * gain);
  

  public void stopTele(){
    m_teleGrip.stopMotor();
  }

  @Override
  public void periodic() {
  }
}