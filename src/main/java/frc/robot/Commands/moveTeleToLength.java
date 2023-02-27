// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.Commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;

// import frc.robot.Subsystems.Arm;
// import frc.robot.Subsystems.Telescope;
// import frc.robot.Subsystems.PIDCalc;


// public class moveTeleToLength extends CommandBase {
//   private Telescope innerTele;
//   private double innerLength;
//   private Arm innerArm;
//   private PIDCalc innerPid;
//   // private Arm m_arm;

//   public moveTeleToLength(Telescope outerTele, Arm outerArm, PIDCalc outerPid, double length) {
//     // this.m_arm = new Arm();
//     this.innerTele = outerTele;
//     this.innerArm = outerArm;
//     this.innerPid = outerPid;
//     this.innerLength = length;
//     addRequirements(innerTele);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     this.innerTele.setSetpointLength(this.length);
//     this.innerPid.setSetpoint(this.length);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     this.innerPid.setInput(this.innerTele.getLength());
//     // if (this.length - this.innerTele.getLength() > 0){
//     //     innerTele.moveTeleManually(0.5, this.innerArm.getAngle()); //this.m_arm.getAngle()
//     // }
//     // else {
//     //     innerTele.moveTeleManually(-0.5, this.innerArm.getAngle());
//     // }
//     this.innerTele.moveTeleToPos();
    
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     innerTele.stopTele();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return innerPid.atSetPoint();
//   }
// }
