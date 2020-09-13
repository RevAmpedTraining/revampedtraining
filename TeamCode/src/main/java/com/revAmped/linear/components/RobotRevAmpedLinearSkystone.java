package com.revAmped.linear.components;

import com.revAmped.components.Drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.revAmped.components.RobotRevAmpedSkystone;
/**
 * Created by swang4 on 6/21/2019.
 */

public class RobotRevAmpedLinearSkystone
        extends RobotRevAmpedSkystone{
    protected LinearOpMode op;
    public final SlideLinear slides;

    public RobotRevAmpedLinearSkystone(LinearOpMode op)
    {
        super(op,
                true);

        this.op = op;
        getMecanumDriveLinear().setLinearOpMode(op);
        slides = new SlideLinear(op,
                null,
                revSwitch,
                vertL,
                vertR);
    }

    @Override
    protected Drive createDrive() {
        return new MecanumDriveLinear(driveLeftFront,
                driveLeftBack,
                driveRightFront,
                driveRightBack,
                gyroSensor,
                sonarF,
                sonarL,
                sonarR);
    }

    public MecanumDriveLinear getMecanumDriveLinear () {
        return (MecanumDriveLinear) this.drive;
    }
}
