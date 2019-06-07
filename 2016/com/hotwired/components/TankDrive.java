package com.revAmped.components;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

/**
 * Drive class for OpMode.
 */
public class TankDrive
    extends Drive
{

    public TankDrive(HwMotor driveLeftFront,
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
    public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {
        driveLeftFront.setZeroPowerBehavior(behavior);
        driveLeftBack.setZeroPowerBehavior(behavior);
        driveRightFront.setZeroPowerBehavior(behavior);
        driveRightBack.setZeroPowerBehavior(behavior);
    }

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
            default:
                break;
        }
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
