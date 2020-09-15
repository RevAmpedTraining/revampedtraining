package org.firstinspires.ftc.teamcode.SampleAutonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.revAmped.components.RobotEncoderTest;
import com.revAmped.components.TurnType;
import com.revAmped.linear.components.MecanumDriveLinear;
import com.revAmped.linear.components.RobotRevAmpedLinearSkystone;
import com.revAmped.linear.util.SelectLinear;


public class JohnAuto extends LinearOpMode {

    private RobotRevAmpedLinearSkystone robot;

    private MecanumDriveLinear drive;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotRevAmpedLinearSkystone(this);
        drive = robot.getMecanumDriveLinear();

        SelectLinear sl = new SelectLinear(this);

        boolean isRed = sl.selectAlliance();
        boolean isHang = sl.selectHang();

        waitForStart();

        //if we want to hang, then we move forward 10 inches to hang, if we don't want to hang, move backward 10 inches to park
        drive.moveToEncoderInch(TurnType.FORWARD, isHang? 10 : -10, 0.5f, 5000, true, true);

        //if blue, turn 90, if red turn -90
        drive.turn(isRed? -90 : 90, 0.5f, 3000, true);

    }
}
