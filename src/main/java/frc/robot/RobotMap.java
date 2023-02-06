package frc.robot;


public class RobotMap {
    
    //https://github.com/wpilibsuite/allwpilib/blob/main/wpilibj/src/main/java/edu/wpi/first/wpilibj/DutyCycleEncoder.java
    /**
    Definition of static variables - ports of driving motor controllers
     */
    public static final int DRIVE_LEFT_MASTER  = 0;
    public static final int DRIVE_LEFT_SLAVE  = 0;
    public static final int DRIVE_RIGHT_MASTER  = 0;
    public static final int DRIVE_RIGHT_SLAVE  = 0;

    /*
     * Definition of joysticks ports.
     */
    public static final int JOYSTICK_CONTROLLER = 0;
    public static final int X_AXIS_PORT = 0;
    public static final int Y_AXIS_PORT = 1;

    /*
     * Definition of gripper solenoids
     */
    public static final int RIGHT_SOLENOID_FW = 2;
    public static final int RIGHT_SOLENOID_BW = 3;
    public static final int LEFT_SOLENOID_FW = 0;
    public static final int LEFT_SOLENOID_BW = 1;

    /*
     * Definition of gripper telescop motor
     */
    public static final int TELESCOPIC_GRIPPER = 8;

    /*
     * Definition of arm motor controllers.
     */

    public static final int ARM_RIGHT_MOTOR = 0;
    public static final int ARM_LEFT_MOTOR = 0;

    /*
     * Definition of arm encoder
     */
    public static final int RIGHT_ENCODER_CHANNEL_A = 1;
    public static final int RIGHT_ENCODER_CHANNEL_B = 2;
    public static final int LEFT_ENCODER_CHANNEL_A = 0;
    public static final int LEFT_ENCODER_CHANNEL_B = 0;
    public static final int ENCODER_TICKS_PER_ROTATION = 2048;

    /*
    Defenition of Xbox Values
    */

    public static final int XBOX_CONTROLLER = 0;
    public static final int PS4_CONTROLLER = 0;
}

