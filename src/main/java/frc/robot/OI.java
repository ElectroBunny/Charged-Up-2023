package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;


public class OI {
    
    //Definiton of joystick buttons
    
    public Joystick joystick_controller = new Joystick(RobotMap.JOYSTICK_CONTROLLER);
    JoystickButton button1 = new JoystickButton(joystick_controller, 1);
    JoystickButton button2 = new JoystickButton(joystick_controller, 2);
    JoystickButton button3 = new JoystickButton(joystick_controller, 3);
    JoystickButton button4 = new JoystickButton(joystick_controller, 4);
    JoystickButton button5 = new JoystickButton(joystick_controller, 5);
    JoystickButton button6 = new JoystickButton(joystick_controller, 6);
    JoystickButton button7 = new JoystickButton(joystick_controller, 7);
    JoystickButton button8 = new JoystickButton(joystick_controller, 8);
    JoystickButton button9 = new JoystickButton(joystick_controller, 9);
    JoystickButton button10 = new JoystickButton(joystick_controller, 10);
    POVButton povbutton1= new POVButton(joystick_controller,0);
    POVButton povbutton2= new POVButton(joystick_controller,180);
    
    //definition of Xbox Controller 

    /*
        Notes for the XBox controller:
        Button A: 1
        Button B: 2
        Button X: 3
        Button Y: 4
        Button LB: 5
        Button RB: 6
        Button LT: Range 0 to 1.000 (stick.GetTrigger())
        Button RT: Range 0 to -1.000 (stick.GetTrigger())
        Button Back: 7
        Button Start: 8
        Left Axis press: 9
                         X-Axis: -1.000 to 1.000 (stick.GetX())
                         Y-Axis: -1.000 to 1.000 (stick.GetY())
        Right Axis press: 10
                         X-Axis: -1.000 to 1.000 (stick.GetTwist())
                         Y-Axis:                         
         */
    
    XboxController xboxController = new XboxController(RobotMap.XBOX_CONTROLLER);
    public JoystickButton A = new JoystickButton(xboxController, 1);
    public JoystickButton B = new JoystickButton(xboxController, 2);
    public JoystickButton X = new JoystickButton(xboxController, 3);
    public JoystickButton Y = new JoystickButton(xboxController, 4);

    public JoystickButton LB = new JoystickButton(xboxController, 5);
    public JoystickButton RB = new JoystickButton(xboxController, 6);
    public JoystickButton LT = new JoystickButton(xboxController, 7);
    public JoystickButton RT = new JoystickButton(xboxController, 8); 

    
    public POVButton povButtonUpXboxLeft = new POVButton(xboxController, 0);
    public POVButton povButtonDownXboxLeft = new POVButton(xboxController, 180);
    public POVButton povButtonUpXboxRight = new POVButton(xboxController, 0);
    public POVButton povButtonDownXboxRight = new POVButton(xboxController, 180);

    //Definition 
    public PS4Controller ps4Controller = new PS4Controller(RobotMap.PS4_CONTROLLER);
    
    public double getJoystickRawAxis(int axis){
        return joystick_controller.getRawAxis(axis);
    }
}