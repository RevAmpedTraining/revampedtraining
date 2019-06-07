package com.revAmped.components;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by swang on 11/30/2017.
 */

public class MecanumDrive
        extends Drive {


    public MecanumDrive(HwMotor driveLeftFront,
                       HwMotor driveLeftBack,
                       HwMotor driveRightFront,
                       HwMotor driveRightBack)
    {
        super(driveLeftFront,
                driveLeftBack,
                driveRightFront,
                driveRightBack);
    }

    /**
     * stop the drive motors
     */
    @Override
    public void stop() {
        setPower(0f);
    }

    /**
     * set drive motor powers for floats
     */
    @Override
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        driveLeftFront.setZeroPowerBehavior(behavior);
        driveLeftBack.setZeroPowerBehavior(behavior);
        driveRightFront.setZeroPowerBehavior(behavior);
        driveRightBack.setZeroPowerBehavior(behavior);
    }

    /**
     * turn the robot at a specified power for each wheel of Mecanum wheels
     * @param pwr_lf motor power left
     * @param pwr_lb motor power right
     * @param pwr_rf motor power left
     * @param pwr_rb motor power right
     *
     *      FORWARD(+x)   SIDEWAYS RIGHT(+y)   TURN RIGHT(+r)
    front left      +                 +                  +
    front right     +                 -                  -
    back left       +                 -                  +
    back right      +                 +                  -
     */
    public void setPower(float pwr_lf,
                         float pwr_lb,
                         float pwr_rf,
                         float pwr_rb
    )
    {

        driveLeftFront.setPower(pwr_lf);
        driveLeftBack.setPower(pwr_lb);
        driveRightFront.setPower(pwr_rf);
        driveRightBack.setPower(pwr_rb);

    }

    public void setStrafePower(float pwrl,
                               float pwrr)
    {
        driveLeftFront.setPower(pwrl);
        driveLeftBack.setPower(-pwrl);
        driveRightFront.setPower(-pwrr);
        driveRightBack.setPower(pwrr);
    }

    public void setTurnPower(float pwr)
    {
        driveLeftFront.setPower(-pwr);
        driveLeftBack.setPower(-pwr);
        driveRightFront.setPower(pwr);
        driveRightBack.setPower(pwr);
    }

   /* public void turnWhileStrafe (float p, float at, float a) {
        driveLeftFront.setPower(p*(Math.cos(at + .7854f)));
        driveLeftBack.setPower(-p);
        driveRightFront.setPower(-p);
        driveRightBack.setPower(p);
    }*/
    //strafe right when positive, left when negative
    public void setStrafePower(float p)
    {
        driveLeftFront.setPower(p);
        driveLeftBack.setPower(-p);
        driveRightFront.setPower(-p);
        driveRightBack.setPower(p);
    }

    /**
     * run the drive motors forward or backward, depending on the sing of p
     * @param p motor power
     */
    public void setPower(float p) {
        driveLeftFront.setPower(p);
        driveLeftBack.setPower(p);
        driveRightFront.setPower(p);
        driveRightBack.setPower(p);
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
        enc[0] = Math.abs(driveLeftFront.getCurrentPosition());
        enc[1] = Math.abs(driveLeftBack.getCurrentPosition());
        enc[2] = Math.abs(driveRightFront.getCurrentPosition());
        enc[3] = Math.abs(driveRightBack.getCurrentPosition());

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



    //=========================below are  API from Drive, not much useful for Mecanum drive.==================
    /**
     * turn the robot at a specified power, ,
     * if pwrl  = pwrr:  go straight
     *    pwrl > 0 && pwrl > pwrr :  turn right
     *    pwrl > 0 && pwrl < pwrr :  turn left
     *    pwrl < 0 && pwrl > pwrr :  turn right
     *    pwrl < 0 && pwrl < pwrr :  turn left
     * @param pwrl motor power left
     * @param pwrr motor power right
     */
    public void setPower(float pwrl,
                         float pwrr) {
        driveLeftFront.setPower(pwrl);
        driveLeftBack.setPower(pwrl);
        driveRightFront.setPower(pwrr);
        driveRightBack.setPower(pwrr);
    }

    /**
     * turn the robot at a specified power,  this is API from Drive, not much useful for Mecanum drive.
     * @param pwrl motor power left
     * @param pwrr motor power right
     * @param turnType turning method to use
     */

    public void setPower(float pwrl,
                         float pwrr,
                         TurnType turnType) {
        switch (turnType) {
            case TURN_REGULAR:
                driveLeftFront.setPower(-pwrl);
                driveLeftBack.setPower(-pwrl);
                driveRightFront.setPower(pwrr);
                driveRightBack.setPower(pwrr);
                break;
            case TURN_RIGHT_PIVOT:
                driveRightFront.setPower(pwrr);
                driveRightBack.setPower(pwrr);
                break;
            case TURN_LEFT_PIVOT:
                driveLeftFront.setPower(-pwrl);
                driveLeftBack.setPower(-pwrl);
                break;
            case STRAFE:
                // right is plus
                //added for strafe 12/10 jw
                driveRightFront.setPower(-pwrl);
                driveLeftFront.setPower(pwrl);
                driveRightBack.setPower(pwrr);
                driveLeftBack.setPower(-pwrr);
                break;
            default:
                break;
        }
    }

}
