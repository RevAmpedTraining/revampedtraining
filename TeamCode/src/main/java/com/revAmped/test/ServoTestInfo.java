package com.revAmped.test;

import com.revAmped.components.HwServo;

/**
 * Created by zwang on 10/30/2017.
 */

public class ServoTestInfo {
    public HwServo servo;
    public int timeScale = 1;
    public float[] positions;

    /**
     * @param servo
     * @param positions servo positions other than the initial position
     */
    public ServoTestInfo(HwServo servo,
                         float... positions) {
        this.servo = servo;
        this.positions = positions;
    }

    /**
     * @param servo
     * @param positions servo positions other than the initial position
     */
    public ServoTestInfo(HwServo servo,
                          int timeScale,
                          float... positions) {
        this(servo, positions);
        this.timeScale = timeScale;
    }
}
