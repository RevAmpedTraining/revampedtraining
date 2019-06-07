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
}
