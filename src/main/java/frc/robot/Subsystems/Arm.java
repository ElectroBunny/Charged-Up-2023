// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Arm extends SubsystemBase {
  //Definition of motor controllers for the Arm system
  private WPI_VictorSPX armRightMotor;
  private WPI_VictorSPX armLeftMotor;

  private ArmPID armPid;
  
  private boolean isRight = true;
  public double setPointAngle;

  
  public Arm() {
    //initializes the arm pid class object
    armPid = new ArmPID();

    //initialize arm motors
    this.armRightMotor = new WPI_VictorSPX(RobotMap.ARM_RIGHT_MOTOR);
    this.armLeftMotor = new WPI_VictorSPX(RobotMap.ARM_LEFT_MOTOR);

    setPointAngle = 0;

    armLeftMotor.setNeutralMode(NeutralMode.Coast);
    armRightMotor.setNeutralMode(NeutralMode.Coast);

    armLeftMotor.setInverted(true);
    armRightMotor.setInverted(false);

    armLeftMotor.follow(armRightMotor);
  }

  // public void moveArmToGrip(double armGain){
  //   armRightMotor.set(armPid.changeSideToGripper(isRight));
  // }

  public void moveArmToAngle(){
    armRightMotor.set(armPid.setToAngle(this.setPointAngle));
  }

  @Override
  public void periodic() {
  }
 
}
