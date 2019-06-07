package org.firstinspires.ftc.teamcode.Test;

import com.revAmped.components.HwLed;
import com.revAmped.components.RobotOvercharged2;
import com.revAmped.linear.components.RobotOverchargedLinear2;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.text.DecimalFormat;

import static com.revAmped.config.RobotOverchargedConstants.TAG_A;

/**
 * Created by Parthiv on 2/24/2018.
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "ReadPictograph", group = "Individual")
public class ReadPictograph extends LinearOpMode {
    private RobotOvercharged2 robot;
    private final static DecimalFormat numberFormatter = new DecimalFormat("######");
    public HwLed ledBlue;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode()
            throws InterruptedException {
        try {
            // init
            robot = new RobotOverchargedLinear2(this);
            RobotLog.ii(TAG_A, "RobotOverchargedLinear initialized");
            run();
        } finally {
            // shut down
            if (robot != null) {
                robot.close();
            }
        }
    }

    public void run()
            throws InterruptedException {

        telemetry.addData("Status", "Running: " + runtime.toString());
        RelicRecoveryVuMark glyphColumnKey = RelicRecoveryVuMark.UNKNOWN;
        try {
            glyphColumnKey = robot.relicRecoveryVuMark.getGlyphColumnKey();
        } catch (Exception e) {}
        telemetry.addData("GlyphColumnKey:", "%s", glyphColumnKey);
        RobotLog.ii(TAG_A, "GlyphColumnKey=%s", glyphColumnKey);
        telemetry.update();
    }
}
