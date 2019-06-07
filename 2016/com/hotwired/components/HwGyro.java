package com.revAmped.components;

/**
 * Created by zwang on 7/15/2016.
 */
public abstract class HwGyro
{
    protected volatile float baseHeading = 0;

    /**
     * get gyro turn heading
     * @return gyro heading
     */
    public abstract float getHeading();

    public abstract float getRawHeading();

    /**
     * reset gyro headings to the current gyro reading
     */
    public abstract void resetHeading();

    public void resetHeading(float baseHeading){
        baseHeading = baseHeading % 360f;
        if (baseHeading > 180) {
            baseHeading = baseHeading - 360;
        }
        this.baseHeading = baseHeading;
    }

    public void adjustHeading(float adjust) {
        float bh = (this.baseHeading + adjust) % 360f;
        if (bh > 180) {
            bh = bh - 360;
        }
        this.baseHeading = bh;
    }

    /**
     * get status of gyro sensor
     * @return if the gyro sensor is still calibrating
     */
    public abstract boolean isCalibrating();

    /**
     * get status of gyro sensor
     * @return if the gyro sensor is connected
     */
    public abstract boolean isConnected();

    /**
     * calibrate the gyro sensor
     * @throws InterruptedException if the thread is interrupted
     */
    public abstract void calibrate() throws InterruptedException;

    public abstract void close ();
}
