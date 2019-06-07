package com.revAmped.linear.components;

import com.revAmped.components.Drive;
import com.revAmped.components.RobotRevAmped2;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This class is for Autonomous
 */
public class RobotRevAmpedLinear2
        extends RobotRevAmped2
{
    public final SlideLinear slide;
    protected LinearOpMode op;

    public RobotRevAmpedLinear2(LinearOpMode op)
    {
        super(op,
              true);

        this.op = op;
        getMecanumDriveLinear().setLinearOpMode(op);
        slide = new SlideLinear(op,
                                switchSlideUp,
                                switchSlideDown,
                                motorSlide);

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

    public MecanumDriveLinear getMecanumDriveLinear () {
        return (MecanumDriveLinear)this.drive;
    }

}
