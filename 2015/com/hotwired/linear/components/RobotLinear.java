package com.revAmped.linear.components;

import com.revAmped.components.Drive;
import com.revAmped.components.Robot;
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
        getTankDriveLinear().setLinearOpMode(op);
    }

    @Override
    protected Drive createDrive() {
        return new TankDriveLinear(driveLeftFront,
                                   driveLeftBack,
                                   driveRightFront,
                                   driveRightBack,
                                   gyroSensor);
    }

    public TankDriveLinear getTankDriveLinear () {
        return (TankDriveLinear)this.drive;
    }

}
