// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIDCalc extends SubsystemBase{

  /*
  This class is destined to control the motors by PID controller. 
  */
  private PIDController pid;
  private double dataInput;
  private double setpoint;
  private double error;
  private double output;
  private double tolerance;
  /*
  Constructor + reset of pid values from previous runs
  */
  public PIDCalc(double kP, double kI, double kD, double tolerance) {
      pid = new PIDController(kP, kI, kD);
      pid.reset();
      this.tolerance = tolerance;
      pid.setTolerance(tolerance);
    }
  
    // Setters and Getters
  
    /**Gets input from the sensor and sets it as the current measures for the PID calculations.
      *
      * @param input the current measure from the sensor. 
    */
  public void setInput(double input){
    this.dataInput = input;
  }

  /**Gets setpoint and sets it as the current setpoint for the PID calculations. 
    *
    * @param setpoint the setpoint for the PID calculation.
  */
  public void setSetpoint(double setpoint){
    this.setpoint = setpoint;
    // SmartDashboard.putNumber("this setpoint", this.setpoint);
  }

  /**Returns the voltage percent output for the motors according to the PID calculations.
   *
   * @return voltage in precents to set the motor to.
   */
  // public double getOutput(){
  //   this.output = MathUtil.clamp(pid.calculate(this.dataInput, this.setpoint), -0.6, 0.6);
  //   return this.output;
  // }

  /**Gets input from the sensor measure and setpoint, sets them in the class attributes, and returns voltage in precent as output.
    * @param input the current measure from the sensor.
    * @param setpoint the setpoint for the PID calculation.
    * @return voltage in precents to set the motor to.
   */
  public double getOutput(){ //double input, double setpoint
    this.output = MathUtil.clamp(pid.calculate(this.dataInput, this.setpoint), -0.8, 0.8);
    return this.output;
  }

  /**Returns if the system has reached its setpoint.
   * 
   * @return True if the system has reached its setpoint. Otherwise, false.
   */
  public boolean atSetPoint(){
    this.error = this.setpoint - this.dataInput;
    // SmartDashboard.putNumber("Error", this.error);
    // SmartDashboard.putNumber("Tolerance", this.tolerance);
    // SmartDashboard.putBoolean("Condition", Math.abs(this.error) <= this.tolerance);
    // SmartDashboard.putNumber("Setpoint", this.setpoint);

    if (Math.abs(this.error) <= this.tolerance){
      return true;
    }
    else {
      return false;
    }
  }
  
  /**Resets the PID last error and integral value */
  public void resetPID(){
    pid.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
