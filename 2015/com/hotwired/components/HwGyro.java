package com.revAmped.components;

/**
 * Created by zwang on 7/15/2016.
 */
public interface HwGyro
{
    /**
     * get gyro turn heading
     * @return gyro heading
     */
    int getHeading();

    /**
     * reset gyro headings
     */
    void resetHeading();

    /**
     * get status of gyro sensor
     * @return if the gyro sensor is still calibrating
     */
    boolean isCalibrating();

    /**
     * calibrate the gyro sensor
     * @throws InterruptedException if the thread is interrupted
     */
    void calibrate() throws InterruptedException;
}
