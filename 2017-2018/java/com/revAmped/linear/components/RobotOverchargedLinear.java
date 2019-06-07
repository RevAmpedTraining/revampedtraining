package com.revAmped.linear.components;

import com.revAmped.components.Drive;
import com.revAmped.components.RobotOvercharged;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This class is for Autonomous
 */
public class RobotOverchargedLinear
    extends RobotOvercharged
{
    public final SlideLinear slide;
    protected LinearOpMode op;

    public RobotOverchargedLinear(LinearOpMode op)
    {
        super(op,
              true);

        this.op = op;
        getSwerveDriveLinear().setLinearOpMode(op);
        slide = new SlideLinear(op,
                                switchSlideUp,
                                switchSlideDown,
                                slideLeft,
                                slideRight);
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
                                     null/*sonarFront*/,
                                     null/*sonarLeft*/,
                                     null/*sonarRight*/,
                                     null/*colorSensor*/);
    }

    public SwerveDriveLinear getSwerveDriveLinear () {
        return (SwerveDriveLinear)this.drive;
    }

}
