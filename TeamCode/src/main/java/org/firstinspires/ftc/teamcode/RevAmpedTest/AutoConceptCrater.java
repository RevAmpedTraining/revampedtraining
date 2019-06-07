package org.firstinspires.ftc.teamcode.RevAmpedTest;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.revAmped.components.HwLed;
import com.revAmped.components.TurnType;
import com.revAmped.config.RobotRevAmpedConstants;
import com.revAmped.linear.components.RobotRevAmpedLinearTest;
import com.revAmped.linear.components.SwerveDriveLinear;
import com.revAmped.linear.util.SelectLinear;
import com.revAmped.linear.util.WaitLinear;
import com.revAmped.sensors.VuMarkSensing;
import com.revAmped.util.SampleOrderDetector;

import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

/**
 * Created by swang4 on 2/20/2019.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Crater Concept", group = "Test")
@Disabled
public class AutoConceptCrater extends LinearOpMode {

    private RobotRevAmpedLinearTest robot;

    private SwerveDriveLinear drive;

    public void runOpMode() {
        robot = new RobotRevAmpedLinearTest(this);
        drive = robot.getSwerveDriveLinear();
        try {
            run();
        } catch (Exception e) {
            telemetry.addLine("Stop Requested");
            telemetry.update();
            robot.close();
        } finally {
            robot.close();
        }
    }


    public void run()
            throws InterruptedException {
        try {
            if (!isStopRequested()) {
                //selecting auto modes
                VuMarkSensing vuSensor = new VuMarkSensing(this.hardwareMap);
                //initialize vuforia
                Boolean bInit = vuSensor.initialize();
                if (bInit) {
                    telemetry.addData("Vuforia", "Initalized");
                } else {
                    telemetry.addData("Vuforia", "Failed to Init");
                }
                SelectLinear sl = new SelectLinear(this);
                boolean isCrater = sl.selectPosition();
                boolean isSample = sl.selectSample();
                telemetry.addData("Crater", isCrater ? "Yes" : "No");
                telemetry.addData("isSample?", isSample ? "Yes" : "No");
                robot.ledGreen.on();
                robot.gyroSensor.resetHeading();
                WaitLinear lp = new WaitLinear(this);

                //stall until play is pressed
                while (!opModeIsActive() && !isStopRequested()) {
                    telemetry.addData("status", "waiting for start command...");
                    telemetry.update();
                }
                robot.ledRed.off();
                robot.ledBlue.off();
                robot.ledGreen.off();
                robot.ledWhite.off();
                robot.ledYellow.off();
                robot.drawLed();
                long startAnalyzeTime = System.currentTimeMillis();
                //Sampling
                SampleOrderDetector.GoldenOrder order = SampleOrderDetector.GoldenOrder.UNKNOWN;
                if (isSample) {
                    if (bInit) { // only call OpenCV if bInit is true. otherwise, it caused robot to fail, unless try catch it.
                        Bitmap bitmap = vuSensor.vuforia.getBitmap();
                        Mat origimg = new Mat(bitmap.getHeight(), bitmap.getWidth(), CvType.CV_8UC4);   // 8 bits  3 channel
                        Utils.bitmapToMat(bitmap, origimg);

                        SampleOrderDetector sample = new SampleOrderDetector();

                        order = sample.GetSampleOrder(origimg);
                        telemetry.addData("GOLD", "%s", order);

                        telemetry.update();
                        idle();
                        origimg.release();
                        vuSensor.stop();
                    }
                    if (order == SampleOrderDetector.GoldenOrder.UNKNOWN) {
                        order = SampleOrderDetector.GoldenOrder.RIGHT;
                    }
                    telemetry.addData("Gold", order);
                }
                long endAnalyzeTime = System.currentTimeMillis();
                telemetry.addData("Time to Analyze", -startAnalyzeTime + endAnalyzeTime);
                telemetry.update();
                long startTime = System.currentTimeMillis();
                //Landing
                if (isCrater) {

                } else {
                    drive.moveToEncoderInch(TurnType.FORWARD,
                            45,
                            0.5f,
                            5000,
                            false,
                            false,
                            true);

                }

            }
        } catch (InterruptedException e) {
            robot.close();
        }
    }
}