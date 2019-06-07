package com.revAmped.components;


/**
 * Created by zwang on 7/15/2016.
 */
public abstract class HwGyro
        extends HwDevice
{
    protected volatile float baseHeading = 0;
    protected volatile float basePitch = 0;
    protected volatile float baseRoll = 0;
    protected HwGyro(String id) {
        super(id);
    }
    /**
     * get gyro turn heading
     * @return gyro heading
     */
    public float getHeading() {
        float heading = (getRawHeading() - baseHeading + 360f) % 360f;
        if (heading > 180f) {
            heading = heading - 360f;
        }
        return heading;
    }
    public float getPitch() {
        float pitch = (getRawPitch() - basePitch + 360f)%360f;
        if (pitch > 180f) {
            pitch = pitch - 360f;
        }
        return pitch;
    }
    public float getRoll() {
        float roll = (getRawRoll() - baseRoll + 360f)%360f;
        if (roll > 180f) {
            roll = roll - 360f;
        }
        return roll;
    }
    public abstract float getRawHeading();
    public abstract float getRawRoll();
    public abstract float getRawPitch();

    /**
     * reset gyro headings to the current gyro reading
     */
    public void resetHeading() {
        baseHeading = getRawHeading();
    }

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
     * course adjust the direction
     * @return how much to adjust by
     */
    public float adjustDirection () {

        final float RATIO_SERVO_ADJUST = 30f;

        float deltaHeading = (360f - getHeading()) % 360f;
        if (deltaHeading > 180f) {
            deltaHeading = deltaHeading - 360f;
        }
        float adjust =  deltaHeading / RATIO_SERVO_ADJUST;

        //op.telemetry.addData("delta:", deltaHeading);
        //op.telemetry.addData("adjust:", adjust);

        return adjust;
    }

    /**
     * get status of gyro sensor
     * @return if the gyro sensor is still calibrating
     */
    public abstract boolean isCalibrating();

    /**
     * calibrate the gyro sensor
     * @throws InterruptedException if the thread is interrupted
     */
    public abstract void calibrate() throws InterruptedException;
}
