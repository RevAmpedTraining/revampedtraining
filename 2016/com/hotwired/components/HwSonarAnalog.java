package com.revAmped.components;

import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.revAmped.config.RobotConstants.SONAR;

/**
 * Created by zwang on 11/22/2016.
 */

public class HwSonarAnalog
        extends HwDevice {

    private DeviceInterfaceModule dim;
    private SONAR sonar;
    private double scale = SCALE_MAX_LV;
    private final double maxV;

    public static final float SCALE_MAX_LV = 512.0f;
    public static final float SCALE_MAX_XL = 1024.0f;

    public HwSonarAnalog(DeviceInterfaceModule dim,
                         SONAR sonar,
                         float scale) {
        super("Sonar_" + sonar.toString());
        this.dim = dim;
        this.sonar = sonar;
        maxV = dim.getMaxAnalogInputVoltage();
        this.scale = scale;
    }

    public double getScale() {
        return scale;
    }

    public int getChannel() {
        return sonar.channel;
    }

    /**
     * Send an ultrasonic pulse to measure the distance
     * and return the distance measured
     * @return the distance measured.  Returns 0 if there is an error.
     */
    public float getDistance ()
    {
        double v = dim.getAnalogInputVoltage(sonar.channel);
        return (float)(scale * v / maxV);
    }
}
