package com.revAmped.components;

import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by zwang on 2/18/2016.
 */
public class HwMRGyro
    extends HwGyro {

    private GyroSensor gyro;

    /**
     * initialize the gyro sensor
     * @param hardwareMap HardwareMap to get the sensor from
     * @param id name of gyro sensor
     */
    public HwMRGyro(HardwareMap hardwareMap,
                    String id)
        throws IllegalArgumentException, InterruptedException
    {
        gyro = hardwareMap.gyroSensor.get(id);

        gyro.calibrate();
        do {
            Thread.sleep(50);
        }
        while (gyro.isCalibrating());

        gyro.resetZAxisIntegrator();
    }

    /**
     * calibrate the gyro sensor
     */
    @Override
    public void calibrate() {
        gyro.calibrate();
    }

    @Override
    public void close() {
        gyro.close();
    }

    /**
     * get status of gyro sensor
     * @return if the gyro sensor is still calibrating
     */
    @Override
    public boolean isCalibrating() {
        return gyro.isCalibrating();
    }

    /**
     * get status of gyro sensor
     * @return if the gyro sensor is connected
     */
    @Override
    public boolean isConnected() {
        return true;
    }

    /**
     * get gyro turn heading
     * @return gyro heading
     */
    @Override
    public float getHeading() {
        float heading = (gyro.getHeading() - baseHeading + 360f) % 360f;
        if (heading > 180f) {
            heading = heading - 360f;
        }
        return heading;
    }

    @Override
    public float getRawHeading() {
        return gyro.getHeading();
    }

    /**
     * reset gyro headings
     */
    @Override
    public void resetHeading() {
        baseHeading = gyro.getHeading();
    }
}
