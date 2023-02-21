// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.Subsystems;

// import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.revrobotics.ColorMatch;
// import com.revrobotics.ColorSensorV3;
// import com.revrobotics.ColorSensorV3.RawColor;

// public class colorRecognizer extends SubsystemBase {
//   /** Creates a new colorRecognizer. */

//   private final I2C.Port i2cPort = I2C.Port.kOnboard;
  
//   private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

//   private final ColorMatch m_colorMatcher = new ColorMatch();

//   private final Color kCubeTarget = new Color(0.143, 0.427, 0.429); // not the real values
//   private final Color kConeTarget = new Color(0.197, 0.561, 0.240);

//   public colorRecognizer() {
//     m_colorMatcher.addColorMatch(kCubeTarget);
//     m_colorMatcher.addColorMatch(kConeTarget);
//   }

//   public Color getColor(){
//     return m_colorSensor.getColor();
//   }

//   public byte matchColor(){
//      if (this.m_colorMatcher.matchClosestColor(this.getColor()).color == kCubeTarget){
//         return 1;}
//       else if (this.m_colorMatcher.matchClosestColor(this.getColor()).color == kCubeTarget){
//         return 2;}
//         else{
//           return 0;
//         }
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
// }
