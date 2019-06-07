package com.revAmped.components;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by kevin on 11/24/2017.
 */

public class HwDistanceSensor
        extends HwDevice {
    private DistanceSensor distanceSensor;

    public HwDistanceSensor(HardwareMap hardwareMap, String distanceName) {
        super(distanceName);
        distanceSensor = hardwareMap.get(DistanceSensor.class, distanceName);
    }

    /**
     * Send an ultrasonic pulse to measure the distance
     * and return the distance measured
     * @param unit cm or inch
     * @return the distance measured.
     * Returns DistanceUnit.infinity when a distance reading is not in fact available.
     */
    public Double getDistance(DistanceUnit unit) {

        return distanceSensor.getDistance(unit);
    }

    /**
     * Send an ultrasonic pulse to measure the distance
     * and return the distance measured
     * @return the distance measured.
     * Returns DistanceUnit.infinity when a distance reading is not in fact available.
     */
    public float getDistance ()
    {
        return (float)distanceSensor.getDistance(DistanceUnit.CM);
    }
}
