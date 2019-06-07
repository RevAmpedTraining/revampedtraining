package com.revAmped.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by zwang on 2/7/2016.
 */
public class HwServo
    extends HwDevice {

    private Servo servo;
    float initialPosition;

    /**
     * initialize servos
     * @param hardwareMap HardwareMap to get servos from
     * @param id name of servo
     * @param initialPosition starting servo position
     * @throws IllegalArgumentException exception
     */
    public HwServo(HardwareMap hardwareMap,
                   String id,
                   float initialPosition)
        throws IllegalArgumentException
    {
        super(id);
        this.initialPosition = initialPosition;

        this.servo = hardwareMap.servo.get(id);
        this.servo.setPosition(initialPosition);
    }

    /**
     * get servo
     * @return current servo
     */
    public Servo getServo() {
        return servo;
    }

    /**
     * get servo position
     * @return current servo position
     */
    public float getPosition() {
        return (float)servo.getPosition();
    }

    /**
     * set servo position
     * @param position position to set servo to
     */
    public void setPosition(float position) {
        servo.setPosition(position);
    }

    /**
     * get servo initial position
     * @return initial servo position
     */
    public float getInitialPosition() {
        return initialPosition;
    }

    /**
     * set servo to initial position
     */
    public void setInitialPosition() {
        servo.setPosition(initialPosition);
    }
}