package frc.robot;


public class RobotMap {
    //GENERAL UNITS - CM, DEG, PERCENTAGE

    //Definition of joysticks ports.
    public static final int JOYSTICK_CONTROLLER = 0;
    public static final int X_AXIS_PORT = 0;
    public static final int Y_AXIS_PORT = 1;
    public static final int XBOX_CONTROLLER = 1;

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


    //Drive system constants:

    //Definition of drive motors' ports.
    public static final int DRIVE_LEFT_MASTER  = 3;
    public static final int DRIVE_LEFT_SLAVE  = 1;
    public static final int DRIVE_RIGHT_MASTER  = 4;
    public static final int DRIVE_RIGHT_SLAVE  = 2;

 //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    //General dimensions (robot and field)
    public static final double ROBOT_SHIELD_HEIGHT = 13.5;
    public static final double ROBOT_IDLE_HEIGHT = 75.5;
    public static final double MAX_HEIGHT_ALLOWED = 198;
    public static final int ROBOT_SHIELD_WIDTH = 82;
    public static final double SHIELD_TO_CENTER_JOINT = 5.5;

    //Angles definition
    // beware: angle of arm is reseted at floor level and goes clockwise in comparision to robot's front pannel
    public static final double MIN_ANGLE = 44;
    public static final double MAX_ANGLE = 306.25;
    

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    //Telescope system constants:

    //Definition of telescope motor port.
    public static final int TELESCOPIC_GRIPPER = 7;

    //Gain for telescope opening
    public static final double TELESCOPE_GAIN_REVERSE = -1;
    public static final double TELESCOPE_GAIN = 1;

    //Definition of PID constants for the telescope system.
    public static final double KP_TELE = 0;
    public static final double KI_TELE = 0;
    public static final double KD_TELE = 0;
    public static final double TOLRENCE_TELE = 2.5;
    public static final int TELE_GEAR_RATIO = 1; //Ask mechanic team!
    public static final double TELE_SHAFT_PERMITER = 0; //Check!

    //Definition of telescope encoder.
    public static final int TELE_ENCODER_CHANNEL_A = 3;
    public static final int TELE_ENCODER_CHANNEL_B = 4;

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    //Arm system constants:

    //Definition of arm motor's ports.
    public static final int ARM_RIGHT_MOTOR = 8;
    public static final int ARM_LEFT_MOTOR = 10;
    
    //Definition of PID constants for the arm system.
    public static final double KP_ARM = 0;
    public static final double KI_ARM = 0;
    public static final double KD_ARM = 0;
    public static final double TOLRENCE_ARM = 2.5;

    //Definition of arm encoder
    public static final int ARM_ENCODER_CHANNEL_A = 1;
    public static final int ARM_ENCODER_CHANNEL_B = 2;
    
    public static final int ENCODER_PULSES_PER_ROTATION = 2048;
    
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    
    //Definition of gripper solenoids
    public static final int RIGHT_GRIPPER_SOLENOID_FW = 1;
    public static final int RIGHT_GRIPPER_SOLENOID_BW = 7;
    public static final int LEFT_GRIPPER_SOLENOID_FW = 4;
    public static final int LEFT_GRIPPER_SOLENOID_BW = 5;

 //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    //Autonomus game mode constants:

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
     * Definition of telescope distance opening in each scoring mode
     */
    public static final double TELE_LOW_DISTANCE_FRONT = 0;
    public static final double TELE_MID_DISTANCE_CUBE = 0;
    public static final double TELE_HIGH_DISTANCE_CUBE = 0;

    public static final double TELE_LOW_DISTANCE_BACK = 0;
    public static final double TELE_MID_DISTANCE_CONE = 0;
    public static final double TELE_HIGH_DISTANCE_CONE = 0;

    public static final double TELE_MAX_LENGTH = 0;

    /*
     * Definition of autonomus time constants
     */
    public static final double SIMPLE_AUTO_DRIVE_TIME = 0;
    public static final double ARM_AUTONOMUS_TIME = 0;

 //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------




}

