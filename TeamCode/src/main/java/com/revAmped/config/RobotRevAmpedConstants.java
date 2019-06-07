package com.revAmped.config;

/**
 * Shared robot constants (i.e. switch addreses, servo presets, etc)
 */
public interface RobotRevAmpedConstants
        extends RobotConstants
{

    //servo positions  --

    float POWER_SWEEPER = 1.0f;
    float STOP = 0;

    float SERVO_STICK_IN = 10f/255f;
    float SERVO_DUMPER_CLAW_OUT = 126f/255f;

    float SERVO_RELIC_ELBOW_REST = 125f/255f;

    float SERVO_RELIC_CLAW_REST = 146f/255f;

    float SERVO_JEWEL_IN = 60f/255f;
    float SERVO_DETECT = 132f/255f;

    float SERVO_DOOR_IN = 13f/255f;
    float SERVO_CONTAINER_DOWN = 255/255f;

    float SERVO_MARKER_IN = 60f/255f, SERVO_MARKER_DUMP = 188f/255f;

    float INTAKE_HOLD = 110/255f, INTAKE_BLOCK = 58/255f;

    float LATCH_MOTOR = 1f;

    float SERVO_LATCH_IN =162/255f, SERVO_LATCH_OUT = 87/255f;

    float SERVO_DUMP = 144/255f, SERVO_DUMP_UP = 56/255f, SERVO_DUMP_INIT = 56/255f;
}
