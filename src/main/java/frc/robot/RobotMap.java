package frc.robot;


public class RobotMap {

    //general dimensions (robot and field)

    public static final double ROBOT_SHIELD_HEIGHT = 13.5;
    public static final double ROBOT_IDLE_HEIGHT = 75.5;

    public static final int ROBOT_SHIELD_WIDTH = 82;
    public static final double SHIELD_TO_CENTER_JOINT = 5.5;


    //angles for succesful grid placement
    public static final double ANGLE_CONE_HIGH = 28.4;
    public static final double ANGEL_CONE_MID = 24.35;



    
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
    public static final int ARM_ENCODER_CHANNEL_A = 1;
    public static final int ARM_ENCODER_CHANNEL_B = 2;
    public static final int TELE_ENCODER_CHANNEL_A = 3;
    public static final int TELE_ENCODER_CHANNEL_B = 4;
    public static final int ENCODER_PULSES_PER_ROTATION = 2048;

    /*
     * Definition of PID constants for the arm system.
     */
    public static final double KP_ARM = 0;
    public static final double KI_ARM = 0;
    public static final double KD_ARM = 0;
    public static final double TOLRENCE_ARM = 2.5;
    //Angles definition
    // beware: angle of arm is reseted at floor level and goes clockwise in comparision to robot's front pannel
    public static final double MIN_ANGLE = 14;
    public static final double MAX_ANGLE = 306.25;
    /*
    average between range of deviation
    Δα = ±2°
    */
    public static final double LOW_FRONT_ANGLE = 27.1; // 90 - 52.1
    // devitation = +-3.585
    public static final double LOW_BACK_ANGLE = 301.23; // 270 + 52.1
    // devitation = +-3.585
    public static final double MID_CONE_ANGLE = 90 + 24.35;
    // devitation = +- 1.4
    public static final double MID_CUBE_ANGLE = 90 - 0.38;
    // deviation = +-0.02
    public static final double HIGH_CONE_ANGLE = 90 + 27.4;
    // devitation = +-1
    public static final double HIGH_CUBE_ANGLE = 90 + 14.0;
    // decitation = +-0.57

    /*
     * Definition of PID constants for the telescop system.
     */
    public static final double KP_TELE = 0;
    public static final double KI_TELE = 0;
    public static final double KD_TELE = 0;
    public static final double TOLRENCE_TELE = 2.5;

    /*
     * Definition of autonomus time constants
     */
    public static final double SIMPLE_AUTO_DRIVE_TIME = 0;
    public static final double ARM_AUTONOMUS_TIME = 0;
    
    /*
     * Definition of telescop distance opening in each scoring mode
     */
    public static final double TELE_LOW_DISTANCE_FRONT = 0;
    public static final double TELE_MID_DISTANCE_CUBE = 0;
    public static final double TELE_HIGH_DISTANCE_CUBE = 0;

    public static final double TELE_LOW_DISTANCE_BACK = 0;
    public static final double TELE_MID_DISTANCE_CONE = 0;
    public static final double TELE_HIGH_DISTANCE_CONE = 0;



    /*
    Defenition of Xbox Values
    */

    public static final int XBOX_CONTROLLER = 0;
    public static final int PS4_CONTROLLER = 0;
}

