package com.revAmped.linear.components;

import com.revAmped.components.Drive;
import com.revAmped.components.Robot;
import com.revAmped.config.RobotConstants.COLOR_SENSOR;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * This class is for Autonomous
 */
public class RobotLinear
    extends Robot
{
    protected LinearOpMode op;

    public RobotLinear(LinearOpMode op)
    {
        super(op,
              true);

        this.op = op;
        getSwerveDriveLinear().setLinearOpMode(op);
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
                                     sonarFront,
                                     sonarLeft,
                                     sonarRight,
                                     colorSensor);
    }

    public SwerveDriveLinear getSwerveDriveLinear () {
        return (SwerveDriveLinear)this.drive;
    }

}
