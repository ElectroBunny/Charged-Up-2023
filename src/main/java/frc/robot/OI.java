package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI {
    
    public Joystick joystick_controller = new Joystick(RobotMap.JOYSTICK_CONTROLLER);
    

    public double getJoystickRawAxis(int axis){
        return joystick_controller.getRawAxis(axis);
    }
}
