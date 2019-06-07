package com.revAmped.components;

import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by zwang on 2/18/2016.
 */
public class HwMRGyro
    extends HwDevice
    implements HwGyro {
    private int baseHeading = 0;

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
        super(id);

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

    /**
     * get status of gyro sensor
     * @return if the gyro sensor is still calibrating
     */
    @Override
    public boolean isCalibrating() {
        return gyro.isCalibrating();
    }

    /**
     * get gyro turn heading
     * @return gyro heading
     */
    @Override
    public int getHeading() {
        int heading = (gyro.getHeading() - baseHeading + 360) % 360;
        if (heading > 180) {
            heading = heading - 360;
        }
        return heading;
    }

    /**
     * reset gyro headings
     */
    @Override
    public void resetHeading() {
        baseHeading = gyro.getHeading();
    }
}
