package frc.robot;


public class RobotMap {
    //GENERAL UNITS - CM, DEG, PERCENTAGE

    //Definition of joysticks ports.
    public static final int JOYSTICK_CONTROLLER = 0;
    public static final int X_AXIS_PORT = 0;
    public static final int Y_AXIS_PORT = 1;
    public static final int XBOX_CONTROLLER = 1;
    public static final int PS4_CONTROLLER = 2;

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    //Drive system constants:

    //Definition of drive motors' ports.
    public static final int DRIVE_LEFT_MASTER  = 3;
    public static final int DRIVE_LEFT_SLAVE  = 1;
    public static final int DRIVE_RIGHT_MASTER  = 7;
    public static final int DRIVE_RIGHT_SLAVE  = 20;

 //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    //General dimensions (robot and field)
    public static final double ROBOT_SHIELD_HEIGHT = 13.5;
    public static final double ROBOT_IDLE_HEIGHT = 75.5;
    public static final double MAX_HEIGHT_ALLOWED = 198;
    public static final int ROBOT_SHIELD_WIDTH = 68;
    public static final int ROBOT_SHIELD_LENGTH = 82;
    public static final double SHIELD_TO_CENTER_JOINT = 5.5;

    //Angles limitaion definition
    // beware: angle of arm is reseted at floor level and goes clockwise in comparision to robot's front pannel
    public static final double MIN_ANGLE = 50;
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
    public static final double TELE_SHAFT_PERMITER = 9.36;

    //Definition of telescope encoder.
    public static final int TELE_ENCODER_CHANNEL_A = 6;
    public static final int TELE_ENCODER_CHANNEL_B = 7;

//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    //Arm system constants:

    //Definition of arm motor's ports.
    public static final int ARM_RIGHT_MOTOR = 8;
    public static final int ARM_LEFT_MOTOR = 10;

    public static final double RESIST_VOLT = 0.25;
    
    // Definition of arm raise and lower volt.above
    public static final double ARM_RAISE_VOLT = 0.5;
    public static final double ARM_LOWER_VOLT = 0.05;

    //Definition of PID consants for the arm system.
    public static final double KP_ARM = 0.0087; // Big angle - 0.0087      Low - 0.00965
    public static final double KI_ARM = 0.001; // Big angle - 0.001        Low - 0.0017
    public static final double KD_ARM = 0.0047; // Big angle - 0.0047      Low -  0.0047
    public static final double TOLRENCE_ARM = 2.5;

    //Definition of arm encoder
    public static final int ARM_ENCODER_CHANNEL_A = 1;
    public static final int ARM_ENCODER_CHANNEL_B = 2;
    
    public static final int ENCODER_PULSES_PER_ROTATION = 2048;
    
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    
    //Definition of gripper solenoids
    public static final int RIGHT_GRIPPER_SOLENOID_FW = 0;
    public static final int RIGHT_GRIPPER_SOLENOID_BW = 7;
    public static final int LEFT_GRIPPER_SOLENOID_FW = 4;
    public static final int LEFT_GRIPPER_SOLENOID_BW = 5;

 //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    //Autonomus game mode constants:

    // Inspections angles
    public static final double HORIZONTAL_FIRST_ANGLE = 238.0;
    public static final double HORIZONTAL_SECOND_ANGLE = 301.75;
    public static final double MAX_HORIZONTAL_LENGTH = 120;

    public static final double VERTICAL_FIRST_ANGLE = 148.3;
    public static final double VERTICAL_SECOND_ANGLE = 211.7;
    public static final double MAX_VERTICAL_LENGTH = 117;

    /*
    average between range of deviation
    Δα = ±2°
    */
    public static final double LOW_FRONT_ANGLE = 300.425; 
    public static final double LOW_BACK_ANGLE = 51.93; // 270 + 52.1
    public static final double MID_CONE_ANGLE = 270 - (101.68 - 90);
    public static final double MID_CUBE_ANGLE = 270 - (90 - 90);
    public static final double HIGH_CONE_ANGLE = 243; //270 - (114.217 - 90) - 10
    public static final double HIGH_CUBE_ANGLE = 270 - (104.447 - 90) - 10; // 104.447

    public static final double FEEDER_ANGLE = 107.24;

    /*
     * Definition of complete arm length opening in each scoring mode
     */
    public static final double TELE_MIN_LENGTH = 100; // 92
    public static final double TELE_MAX_LENGTH = 145;


    public static final double LOW_LENGTH_FRONT = 109.6;
    public static final double MID_LENGTH_CUBE = TELE_MIN_LENGTH; //Telescope is closed
    public static final double HIGH_LENGTH_CUBE = 118.24 + 18;

    public static final double LOW_LENGTH_BACK = TELE_MIN_LENGTH; //Telescope is closed
    public static final double MID_LENGTH_CONE = 106.2;
    public static final double HIGH_LENGTH_CONE = 144;



    /*
     * Definition of autonomus time constants
     */
    public static final double SIMPLE_AUTO_DRIVE_TIME = 0;
    public static final double ARM_AUTONOMUS_TIME = 0;

 //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
}

