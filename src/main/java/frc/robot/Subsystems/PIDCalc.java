// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
  
  /** Constructor + reset of pid values from previous runs */
  public PIDCalc(double kP, double kI, double kD, double tolerance) {
      pid = new PIDController(kP, kI, kD);
      pid.reset();
      this.tolerance = tolerance;
      pid.setTolerance(tolerance);
  }
  
  /** Gets input from the sensor and sets it as the current measures for the PID calculations.
    * @param input the current measure from the sensor. 
    */
  public void setInput(double input){
    this.dataInput = input;
  }

  /** Gets setpoint and sets it as the current setpoint for the PID calculations. Also resets the pid. 
    * @param setpoint the setpoint for the PID calculation.
    */
  public void setSetpoint(double setpoint){
    this.setpoint = setpoint;
    this.pid.reset();
  }

  /**Gets input from the sensor measure and setpoint, sets them in the class attributes, and returns voltage in precent as output.
    * @param input the current measure from the sensor.
    * @return voltage in precents to set the motor to.
   */
  public double getOutput(double input){
    this.dataInput = input;
    this.output = MathUtil.clamp(pid.calculate(input, this.setpoint), -0.8, 0.8);
    return this.output;
  }

  /** Checks if the system has reached its setpoint.
    * 
    * @return true if the system has reached its setpoint, otherwise - false.
    */
  public boolean atSetPoint(double input){
    this.dataInput = input;
    this.error = this.setpoint - this.dataInput;

    if (Math.abs(this.error) <= this.tolerance){
      return true;
    }
    else {
      return false;
    }
  }
  
  /** Resets the PID last error and integral value */
  public void resetPID(){
    pid.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
