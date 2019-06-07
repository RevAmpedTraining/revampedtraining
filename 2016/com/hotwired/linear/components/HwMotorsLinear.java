package com.revAmped.linear.components;

import com.revAmped.components.HwDevice;
import com.revAmped.components.HwMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;

/**
 * Created by zwang on 2/22/2016.
 */
public class HwMotorsLinear
    extends HwDevicesLinear
{
    public HwMotorsLinear(LinearOpMode op,
                          HwMotor... motors) {
        super(op, motors);
    }

    public HwMotorsLinear(HwMotor... motors) {
        super(motors);
    }

    /**
     * Stop motors until it is confirmed.
     * @throws InterruptedException
     */
    public void stop()
        throws InterruptedException
    {
        set(new Update() {
            public boolean isUpdated(HwDevice motor) {
                return Math.abs(((HwMotor) motor).getPower()) < 0.005;
            }

            public void update(HwDevice motor) {
                ((HwMotor) motor).setPower(0);
            }
        });
    }

    /**
     * set motor regulation until it is confirmed
     * @param mode motor regulation to set to
     */
    public void setMode (final RunMode mode)
        throws InterruptedException
    {
        set(new Update() {
            public boolean isUpdated(HwDevice motor) {
                return ((HwMotor) motor).getMode() == mode;
            }

            public void update(HwDevice motor) {
                ((HwMotor) motor).setMode(mode);
            }
        });
    }

    /**
     * set motor power float until it is confirmed.
     * @throws InterruptedException
     */
    public void setZeroPowerBehavior(final ZeroPowerBehavior behavior)
        throws InterruptedException
    {
        set(new Update() {
            public boolean isUpdated(HwDevice motor) {
                return ((HwMotor) motor).getZeroPowerBehavior() == behavior;
            }

            public void update(HwDevice motor) {
                ((HwMotor) motor).setZeroPowerBehavior(behavior);
            }
        });
    }

    /**
     * set motor target position until it is confirmed
     * @param position motor target position
     */
    public void setTargetPosition(final int position)
            throws InterruptedException {
        setTargetPosition(position, null);
    }

    /**
     * set motor target position until it is confirmed
     * @param position motor target position
     */
    public void setTargetPosition(final int position,
                                  final HwDevice[] motors)
            throws InterruptedException
    {
        set(new Update() {
                public boolean isUpdated(HwDevice motor) {
                    return Math.abs(((HwMotor) motor).getTargetPosition() - position) < 10;
                }

                public void update(HwDevice motor) {
                    ((HwMotor) motor).setTargetPosition(position);
                }
            },
            motors);
    }
}
