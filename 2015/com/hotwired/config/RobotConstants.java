package com.revAmped.config;

/**
 * Shared robot constants (i.e. switch addreses, servo presets, etc)
 */
public interface RobotConstants {
    int SWITCH_BUCKET_CHANNEL = 5;
    int SWITCH_SLIDE_DOWN_CHANNEL = 6;
    int SWITCH_SLIDE_UP_CHANNEL = 7;

    int LED_YELLOW_CHANNEL = 4;
    int LED_GREEN_CHANNEL = 1;
    int LED_WHITE_CHANNEL = 0;
    int LED_BLUE_CHANNEL = 2;
    int LED_RED_CHANNEL = 3;

    int COLOR_LEFT_OUT_ADDRESS = 0X40;
    int COLOR_LEFT_IN_ADDRESS = 0X44;
    int COLOR_RIGHT_IN_ADDRESS = 0X48;
    int COLOR_RIGHT_OUT_ADDRESS = 0X50;

    float SERVO_BUCKET_TURN_CENTER = 0.5f, SERVO_BUCKET_TURN_LEFT = 0.73f, SERVO_BUCKET_TURN_RIGHT = 0.27f;
    float SERVO_BUCKET_MID_FRONT_LEFT = 0.595f, SERVO_BUCKET_MID_FRONT_RIGHT = 0.394f;
    float SERVO_FDOOR_OPEN = 0.11f, SERVO_FDOOR_EASE = 0.27f, SERVO_FDOOR_CLOSED = 0.63f, SERVO_FDOOR_PUSH = 0.87f;
    float SERVO_BDOOR_PUSH = 0.18f, SERVO_BDOOR_OPEN = 0.775f;
    float SERVO_BUCKET_CLOSED = 0.11f, SERVO_BUCKET_OPEN = 0.76f, SERVO_BUCKET_OPEN_WIDE = 1.0f;
    float SERVO_HOOKR_UP = 1.0f, SERVO_HOOKR_DOWN = 0.07f;
    float SERVO_HOOKL_UP = 0.0f, SERVO_HOOKL_DOWN = 0.94f;
    float SERVO_SLIDE_START = 0.0f, SERVO_SLIDE_END = 0.68f;
    float SERVO_CLIMBER_IN = 1.0f, SERVO_CLIMBER_OUT = 0.1f; // digital servo 5485 calibration: DN/R-172, UP/L172
    float SERVO_WINGL_IN = 0.925f, SERVO_WINGL_OUT = 0.27f;
    float SERVO_WINGR_IN = 0.0f, SERVO_WINGR_OUT = 0.625f;
    float SERVO_BEACON_RIGHT = 0f, SERVO_BEACON_MID = 0.5f, SERVO_BEACON_LEFT = 1.0f;

    /** AndyMark stealth wheel */
    int WHEEL_DIAMETER_CM = 10;

    /** Drive gear ratio */
    float WHEEL_TO_MOTOR_GEAR_RATIO = 84.0f/72.0f;
}
