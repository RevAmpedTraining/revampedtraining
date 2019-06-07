package com.revAmped.linear.components;

import com.revAmped.components.Drive;
import com.revAmped.components.RobotRevAmped;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This class is for Autonomous
 */
public class RobotRevAmpedLinear
        extends RobotRevAmped
{
    public final SlideLinear slide;
    protected LinearOpMode op;

    public RobotRevAmpedLinear(LinearOpMode op)
    {
        super(op,
                true);

        this.op = op;
        getTankDriveLinear().setLinearOpMode(op);
        slide = new SlideLinear(op,
                switchSlideUp,
                switchSlideDown,
                motorSlide
                );
    }

    @Override
    protected Drive createDrive() {
        return new MecanumDriveLinear(driveLeftFront,
                driveLeftBack,
                driveRightFront,
                driveRightBack,
                gyroSensor,
                null,
                null,
                null);
    }

    public TankDriveLinear getTankDriveLinear () {
        return (TankDriveLinear)this.drive;
    }

}
