package com.revAmped.components;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.revAmped.linear.util.WaitLinear;

/**
 * Created by zwang on 11/22/2016.
 */

public class HwSonarAnalog
        extends HwDevice {

    private AnalogInput input;
    private double scale = SCALE_MAX_LV;
    private final double maxV;

    public static final float SCALE_MAX_LV = 512.0f;
    public static final float SCALE_MAX_XL = 1024.0f;

    /**
     * Analog sonar sensor
     * @param hardwareMap
     * @param sonarName sonar name
     * @param scale reading scale
     */
    public HwSonarAnalog(HardwareMap hardwareMap,
                         String sonarName,
                         float scale) {
        super(sonarName);
        input = hardwareMap.get(AnalogInput.class, sonarName);
        maxV = input.getMaxVoltage();
        this.scale = scale;
        RobotLog.vv("MaxVofSonar", "%5.1f", maxV);
    }

    public double getScale() {
        return scale;
    }

    /**
     * Send an ultrasonic pulse to measure the distance
     * and return the distance measured
     * @return the distance measured.  Returns 0 if there is an error.
     */
    public float getDistance ()
    {
        double v;
        double v1 = input.getVoltage();
        try {
            Thread.sleep(20);
        } catch (Exception e) {}
        double v2 = input.getVoltage();
        double begMax = Math.max(v1, v2);
        double begMin = Math.min(v1, v2);
        try {
            Thread.sleep(20);
        } catch (Exception e) {}
        double v3 = input.getVoltage();
        if (v3>begMax) {
            v = (v2+v1)/2;
        } else {
            v = (begMin+v3)/2;
        }
        RobotLog.vv("VolSonar","%5.1f, %5.1f, %5.1f, %5.1f ", v1, v2, v3, v);
        return (float)(scale * v / maxV);
    }
    public float getOpticalDistance()
    {
        double v = input.getVoltage();
        v = 9.98577/(v-.148);
        return (float) (v);
    }
}
