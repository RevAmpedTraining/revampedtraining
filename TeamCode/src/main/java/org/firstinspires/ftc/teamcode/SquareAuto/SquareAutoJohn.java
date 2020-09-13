package org.firstinspires.ftc.teamcode.SquareAuto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.revAmped.components.MecanumDrive;
import com.revAmped.components.TurnType;
import com.revAmped.linear.components.MecanumDriveLinear;
import com.revAmped.linear.components.RobotRevAmpedLinearSkystone;

public class SquareAutoJohn extends LinearOpMode {

    private RobotRevAmpedLinearSkystone robot;
    private MecanumDriveLinear drive;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotRevAmpedLinearSkystone(this);
        drive = robot.getMecanumDriveLinear();

        waitForStart();

        //move in a 10 inch square
        drive.moveToEncoderInch(TurnType.FORWARD,
                10,
                0.5f,
                3000,
                true,
                true);

        for (int i = 0; i < 3; i++) {
            drive.turn(90,
                    0.5f,
                    3000,
                    true);

            sleep(1000);

            drive.moveToEncoderInch(TurnType.FORWARD,
                    10,
                    0.5f,
                    3000,
                    true,
                    true);

        }

    }

}
