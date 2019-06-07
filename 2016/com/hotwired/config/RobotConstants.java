package com.revAmped.config;

/**
 * Shared robot constants (i.e. switch addreses, servo presets, etc)
 */
public interface RobotConstants {

    enum SWITCH {
        BUCKET(5),
        SLIDE_DOWN(6),
        SLIDE_UP(7);
        public int channel; SWITCH(int i) { channel = i; }
    }

    enum LED {
        YELLOW(4),
        GREEN(0),
        WHITE(1),
        BLUE(2),
        RED(3);
        public int channel; LED(int i) { channel = i; }
    }

    enum SONAR {
        FRONT(5),
        LEFT(6),
        RIGHT(7);
        public int channel; SONAR(int i) { channel = i; }
    }

    enum COLOR_SENSOR {
        LOW_RIGHT(0),
        LOW_LEFT(3),
        LEFT_BACK(2),
        RIGHT_BACK(1),
        LEFT_FRONT(5),
        RIGHT_FRONT(4);

        public int channel; COLOR_SENSOR(int i) { channel = i; }
    }

    /* This is the port on the Core Device Interace Module */
    /* in which the navX-Micro is connected.  Modify this  */
    /* depending upon which I2C port you are using.        */
    int NAVX_DIM_I2C_PORT = 0;

    int SWITCH_SLIDE_DOWN_CHANNEL = 6;
    int SWITCH_SLIDE_UP_CHANNEL = 7;

    //servo positions
    float SERVO_LEFTFRONT_START = 12f/255f; //c 6
    float SERVO_LEFTBACK_START = 224f/255f; // c 1
    float SERVO_RIGHTFRONT_START = 232f/255f; // c 1
    float SERVO_RIGHTBACK_START = 22f/255f; // c6

    float SERVO_LEFTFRONT_END = 227f/255f; //c 6
    float SERVO_LEFTBACK_END = 13f/255f;  // c 1
    float SERVO_RIGHTFRONT_END = 24f/255f; // c 1
    float SERVO_RIGHTBACK_END = 249f/255f; // c6

    float SERVO_BEACON_OUT = 0f/255f, SERVO_BEACON_IN = 255f/255f;
    float SERVO_BEACON_STOP = 128f/255f;
    float SERVO_TRIGGER_IN = 165f/255f, SERVO_TRIGGER_OUT = 90f/255f;
    float SERVO_CAP_BALL_HOLD = 10f/255f, SERVO_CAP_BALL_OPEN = 87f/255f, SERVO_CAP_BALL_PUSH = 200f/255f;


    /** AndyMark stealth wheel */
    int WHEEL_DIAMETER_CM = 10;

    /** Drive gear ratio */
    float WHEEL_TO_MOTOR_GEAR_RATIO = 1f; //35f/64f;

    float POWER_SPINNER = 0.80f;
    float POWER_ROLLER = 0.9f;
    float POWER_SLIDE = 0.85f;
}
