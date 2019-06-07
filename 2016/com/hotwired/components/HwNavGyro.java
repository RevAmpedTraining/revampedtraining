package com.revAmped.components;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

/**
 * Created by zwang on 2/18/2016.
 */
public class HwNavGyro
    extends HwGyro {

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    private AHRS gyro;

    /**
     * initialize the gyro sensor
     * @param dim Device interface module
     */
    public HwNavGyro(DeviceInterfaceModule dim,
                     int port)
        throws IllegalArgumentException, InterruptedException
    {
        gyro = AHRS.getInstance(dim,
                                port,
                                AHRS.DeviceDataType.kProcessedData);

        do {
            Thread.sleep(50);
        }
        while (gyro.isCalibrating());

        gyro.zeroYaw();
    }

    /**
     * calibrate the gyro sensor
     */
    @Override
    public void calibrate() {}

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
        return gyro.isConnected();
    }

    @Override
    public void close ()
    {
        gyro.close();
    }

    /**
     * get gyro turn heading
     * @return gyro heading
     */
    @Override
    public float getHeading() {
        float heading = (gyro.getYaw() - baseHeading + 360f) % 360f;
        if (heading > 180f) {
            heading = heading - 360f;
        }
        return heading;
    }

    @Override
    public float getRawHeading() {
        return gyro.getYaw();
    }

    /**
     * reset gyro headings
     */
    @Override
    public void resetHeading() {
        baseHeading = gyro.getYaw();
    }
}
