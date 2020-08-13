package com.revAmped.config;

/**
 * Shared robot constants (i.e. switch addreses, servo presets, etc)
 */
public interface RobotConstants {


    class COLOR_SENSOR {
        public int channel;
        public COLOR_SENSOR(int i) { channel = i; }
    }

    /** AndyMark stealth wheel */
    int WHEEL_DIAMETER_CM = 10;

    /** Drive gear ratio */
    float WHEEL_TO_MOTOR_GEAR_RATIO = 0.5f; //35f/64f;

    float POWER_SLIDE = 0.85f;
}
