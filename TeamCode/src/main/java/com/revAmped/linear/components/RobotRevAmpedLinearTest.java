package com.revAmped.linear.components;

import com.revAmped.components.Drive;
import com.revAmped.components.RobotRevAmped;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.revAmped.components.RobotRevampedTest;

/**
 * This class is for Autonomous
 */
public class RobotRevAmpedLinearTest
        extends RobotRevampedTest
{
    // public final SlideLinear slide;
    protected LinearOpMode op;

    public RobotRevAmpedLinearTest(LinearOpMode op)
    {
        super(op,
                true);

        this.op = op;
        getSwerveDriveLinear().setLinearOpMode(op);
        /*slide = new SlideLinear(op,
                switchSlideUp,
                switchSlideDown,
                motorLatch
                );*/
    }

    @Override
    protected Drive createDrive() {
        return new SwerveDriveLinear(driveLeftFront,
                driveLeftBack,
                driveRightFront,
                driveRightBack,
                servoLeftFront,
                servoLeftBack,
                servoRightFront,
                servoRightBack,
                gyroSensor,
                null,
                null,
                null,
                null,
                servoTelescopeR,
                servoTelescopeL,
                motorLatch,
                motorPopper,
                motorIntake,
                switchSlideDown);
    }

    public TankDriveLinear getTankDriveLinear () {
        return (TankDriveLinear)this.drive;
    }
    public SwerveDriveLinear getSwerveDriveLinear () {
        return (SwerveDriveLinear)this.drive;
    }

}
