package com.revAmped.components;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Common methods for robot drive
 */
public abstract class Drive {

    public final HwMotor driveLeftFront;
    public final HwMotor driveLeftBack;
    public final HwMotor driveRightFront;
    public final HwMotor driveRightBack;

    public Drive(HwMotor driveLeftFront,
                 HwMotor driveLeftBack,
                 HwMotor driveRightFront,
                 HwMotor driveRightBack)
    {
        this.driveLeftFront = driveLeftFront;
        this.driveLeftBack = driveLeftBack;
        this.driveRightFront = driveRightFront;
        this.driveRightBack = driveRightBack;
    }

    /**
     * convert inches to encoder ticks
     * @param inch inches to be converted
     * @return encoder ticks for the given inches
     */
    public int inchToTick(int inch) {
        return (int)(inch * Robot.AM40_ENCODER_TICK_PER_INCH);
    }

    /**
     * convert encoder ticks to inches
     * @param ticks encoder ticks to be converted
     * @return inches for the given encoder ticks
     */
    public int tickToInch(int ticks) {
        return (int)(ticks / Robot.AM40_ENCODER_TICK_PER_INCH);
    }

    /**
     * reset the encoder values of the drive motors
     */
    public void resetPosition() {
        driveLeftFront.resetPosition();
        driveLeftBack.resetPosition();
        driveRightFront.resetPosition();
        driveRightBack.resetPosition();
    }

    /**
     * stop the drive motors
     */
    public abstract void stop()
        throws InterruptedException;

    /**
     * run the drive motors
     * @param p motor power
     */
    public void setPower(float p) {
        driveLeftFront.setPower(p);
        driveLeftBack.setPower(p);
        driveRightFront.setPower(p);
        driveRightBack.setPower(p);
    }

    /**
     * turn the robot at a specified power
     * @param p motor power
     * @param turnType turning method to use
     */
    public void setTurn(float p,
                        TurnType turnType) {
        switch (turnType) {
            case TURN_REGULAR:
                driveLeftFront.setPower(-p);
                driveLeftBack.setPower(-p);
                driveRightFront.setPower(p);
                driveRightBack.setPower(p);
                break;
            case TURN_RIGHT_PIVOT:
                driveRightFront.setPower(p);
                driveRightBack.setPower(p);
                break;
            case TURN_LEFT_PIVOT:
                driveLeftFront.setPower(-p);
                driveLeftBack.setPower(-p);
                break;
            default:
                break;
        }
    }

   /**
     * set drive motor powers for floats
     */
    public abstract void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior)
        throws InterruptedException;

    /**
     * check if the drive is busy
     */
    public boolean isBusy() {
        return driveLeftFront.isBusy() || driveLeftBack.isBusy() ||
            driveRightFront.isBusy() || driveRightBack.isBusy();
    }

    /**
     * get the 2nd largest encoder value of drives (2nd largest will most likely be accurate)
     * use bubble sort
     * @return the second largest encoder value of drives
     */
    public int getEncoder() {

        final int NUM_WHEELS = 4;
        // 2nd biggest
        final int RETURN_INDEX = 1;

        int enc[] = new int[NUM_WHEELS];
        enc[0] = driveLeftFront.getCurrentPosition();
        enc[1] = driveLeftBack.getCurrentPosition();
        enc[2] = driveRightFront.getCurrentPosition();
        enc[3] = driveRightBack.getCurrentPosition();

        // bubble sort
        for (int i = 0; i <= RETURN_INDEX; i++)
        {
            for (int j = i + 1; j < NUM_WHEELS; j++)
            {
                if (Math.abs(enc[i]) < Math.abs(enc[j]))
                {
                    // swap
                    int c = enc[i];
                    enc[i] = enc[j];
                    enc[j] = c;
                }
            }
        }

        return enc[RETURN_INDEX];
    }
}