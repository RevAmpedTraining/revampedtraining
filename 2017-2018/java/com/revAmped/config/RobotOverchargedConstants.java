package com.revAmped.config;

/**
 * Overcharged robot constants (i.e. switch addreses, servo presets, etc)
 */
public interface RobotOverchargedConstants
        extends RobotConstants
{

    COLOR_SENSOR LOW_RIGHT = new COLOR_SENSOR(0);
    COLOR_SENSOR LOW_LEFT = new COLOR_SENSOR(3);
    COLOR_SENSOR LEFT_BACK = new COLOR_SENSOR(2);
    COLOR_SENSOR RIGHT_BACK = new COLOR_SENSOR(1);
    COLOR_SENSOR LEFT_FRONT = new COLOR_SENSOR(5);
    COLOR_SENSOR RIGHT_FRONT = new COLOR_SENSOR(4);
    ///Overcharged TAG for log parsing
    String TAG_R = "Overcharged Robot::";
    ///Overcharged TAG for Autonomous log parsing
    String TAG_A = "Overcharged Auto::";
    ///Overcharged TAG for Teleop log parsing
    String TAG_T = "Overcharged TeleOp::";
    ///Overcharged TAG for Diagnostics log parsing
    String TAG_D = "Overcharged Diag::";
    ///Overcharged TAG for Setup log parsing
    String TAG_S = "Overcharged Setup::";

    float SERVO_CLAW_RIGHT_MID = 135f/255f;
    float SERVO_CLAW_LEFT_MID = 127f/255f;
    float SERVO_CLAW_RIGHT_IN = 172f/255f, SERVO_CLAW_RIGHT_OUT = 99f/255f;
    float SERVO_CLAW_LEFT_IN = 160f/255f, SERVO_CLAW_LEFT_OUT = 88f/255f;
	///Jewel arm In Out positions encoder value
    float SERVO_JEWEL_IN = 15f/255f, SERVO_JEWEL_OUT = 199f/255f;
	///Jewel Knocker positions encoder value
    float SERVO_JEWEL_KOCKER_LEFT = 156f/255f, SERVO_JEWEL_KOCKER_MID = 126f/255f, SERVO_JEWEL_KOCKER_RIGHT = 84f/255f;
    ///Slide power constant
    float OVERCHARGED_POWER_SLIDE = 0.95f;
    ///Relic Elbow constants
    float SERVO_RELIC_ELBOW_UP = 152f/255f, SERVO_RELIC_ELBOW_DOWN = 203f/255f, SERVO_RELIC_ELBOW_ANGLED = 198f/255f;
    float SERVO_RELIC_ELBOW_REST = 126f/255f;
    ///Relic Claw constants
    float SERVO_RELIC_CLAW_PREPARE_HOLD = 168f/255f, SERVO_RELIC_CLAW_HOLD = 131f/255f, SERVO_RELIC_CLAW_PREPARE_RELEASE = 145f/255f;
    float SERVO_RELIC_CLAW_REST = 107f/255f, SERVO_RELIC_CLAW_RELEASE = 199f/255f;
    ///Relic power constant
    float OVERCHARGED_POWER_RELIC = 0.85f;

    /// Slide motor max encoder value
    float SLIDE_ENCODER_MAX = 2900;
	///spacing constant for slide positioning
    int SLIDE_SPACING = 100;
	///Slide leve 2 encoder reading
    int SLIDE_LEVEL_TWO = 1500;
	///Slide leve 3 encoder reading
    int SLIDE_LEVEL_THREE = 2080;
	///Slide leve 4 encoder reading
    int SLIDE_LEVEL_FOUR = 3082;
	///Dumper servo container positions encoder value
    float SERVO_CONTAINER_NORMAL = 105f/255f, SERVO_CONTAINER_SCORE = 230f/255f, SERVO_CONTAINER_TILT = 135f/255f, SERVO_CONTAINER_LIMIT = 236f/255f;
}