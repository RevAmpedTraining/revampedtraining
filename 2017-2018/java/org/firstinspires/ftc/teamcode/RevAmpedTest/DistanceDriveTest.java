package org.firstinspires.ftc.teamcode.LastYear.RevAmpedTest;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.revAmped.components.Button;
import com.revAmped.components.HwSonarAnalog;
import com.revAmped.components.TurnType;
import com.revAmped.config.RobotRevAmpedConstants;
import com.revAmped.linear.components.RobotRevAmpedLinear;
import com.revAmped.linear.components.RobotRevAmpedLinear2;
import com.revAmped.linear.components.MecanumDriveLinear;
import com.revAmped.linear.util.SelectLinear;
import com.revAmped.linear.util.WaitLinear;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by swang4 on 8/13/2018.
 */

public class DistanceDriveTest
        extends LinearOpMode {
    private RobotRevAmpedLinear2 robot = null;

    private MecanumDriveLinear drive;

    private ElapsedTime runtime = new ElapsedTime();

    private HwSonarAnalog sonarTest;

    private HwSonarAnalog opticalTest;

    @Override
    public void runOpMode()
        throws InterruptedException {
        robot = new RobotRevAmpedLinear2(this);
        drive = robot.getMecanumDriveLinear();
        sonarTest = new HwSonarAnalog(hardwareMap,
                "sonar_test",
                HwSonarAnalog.SCALE_MAX_XL);
        opticalTest = new HwSonarAnalog(hardwareMap,
                "optical_test",
                HwSonarAnalog.SCALE_MAX_XL);
        waitForStart();
        runtime.reset();
        Boolean isOptical = false;
        while (opModeIsActive()) {
            float distanceSonar = sonarTest.getDistance();
            if (distanceSonar <13) {
                distanceSonar = opticalTest.getOpticalDistance();
                isOptical = true;
            }
            if (distanceSonar < 25 && isOptical) {
                drive.setPower(0.32f, 0, 0, 0.32f);
            } else if (distanceSonar > 20) {
                drive.setPower(-0.32f, 0, 0, -0.32f);
            }
        }
    }



}
