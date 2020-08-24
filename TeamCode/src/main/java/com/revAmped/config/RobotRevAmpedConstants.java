package com.revAmped.config;

/**
 * Shared robot constants (i.e. switch addreses, servo presets, etc)
 */
public interface RobotRevAmpedConstants
        extends RobotConstants
{
    float STOP = 0;
    float START_SERVO = 0;
    float END_SERVO = 255/255f;
    //motor constants
    float POWER_SPINNER = 0.96f;
    float POWER_SLIDE = 0.4f;
    //servo positions  --
    //test
    float H_IN = 1/255f, H_MID1 = 164/255f, H_MID2 = 185/255f, H_OUT = 230/255f;

    float CLAW_MID = 8/255f, CLAW_RIGHT = 85/255f, AUTO_TURN = 18/255f;

    float DUMP_AUTO = 29/255f, DUMP_START = 150/255f, DUMP_CLOSE = 186/255f;

    float LATCH_INL = 40/255f, LATCH_OUTL = 158/255f, LATCH_INR = 182/255f, LATCH_OUTR = 65/255f;

    float CAP_START = 179/255f, CAP_OUT = 236/255f;

    float STOPPER = 60/255f, UP = 2/255f;



    //past constants
    float SERVO_STICK_IN = 10f/255f;
    float SERVO_DUMPER_CLAW_OUT = 126f/255f;
    float SERVO_RELIC_ELBOW_REST = 125f/255f;
    float SERVO_RELIC_CLAW_REST = 146f/255f;
    float SERVO_JEWEL_IN = 60f/255f, SERVO_DETECT = 132f/255f;
    float SERVO_DOOR_IN = 13f/255f;
    float SERVO_CONTAINER_DOWN = 255/255f;
    float SERVO_MARKER_IN = 60f/255f, SERVO_MARKER_DUMP = 188f/255f;
    float LATCH_MOTOR = 1f;
    float SERVO_LATCH_IN =162/255f, SERVO_LATCH_OUT = 87/255f;
    float SERVO_DUMP = 144/255f, SERVO_DUMP_UP = 56/255f, SERVO_DUMP_INIT = 56/255f;
    //initial setup.
}
