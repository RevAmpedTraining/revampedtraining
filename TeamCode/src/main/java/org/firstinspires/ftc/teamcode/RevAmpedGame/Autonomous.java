package org.firstinspires.ftc.teamcode.RevAmpedGame;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.revAmped.components.HwLed;
import com.revAmped.components.TurnType;
import com.revAmped.config.RobotRevAmpedConstants;
import com.revAmped.linear.components.RobotRevAmpedLinearTest;
import com.revAmped.linear.components.SwerveDriveLinear;
import com.revAmped.linear.util.SelectLinear;
import com.revAmped.linear.util.WaitLinear;
import com.revAmped.util.MoveActionImpl;
import com.revAmped.util.SampleOrderDetector;

import android.graphics.Bitmap;

import com.revAmped.sensors.VuMarkSensing;

import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

/**
 * Created by John Wang on 10/27/2018.
 */
//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Six Glyph Auto", group="Game")
public class Autonomous
        extends LinearOpMode {
    //initialize the robot objects

    private RobotRevAmpedLinearTest robot;

    private SwerveDriveLinear drive;

    private final float gearFactor = .66f;

    //how much to drive forward to double sample
    private final int doubleSampleDrive = 20;
    //how much to strafe for double sample
    private final int doubleSampleStrafe = -10;
    //how far away from depot in crater position
    private final int distanceToDepot = 65;

    final HwLed.ELedStatus[] ledStatuses = new HwLed.ELedStatus[]{
            HwLed.ELedStatus.LED_ON,
            HwLed.ELedStatus.LED_BLINK,
            HwLed.ELedStatus.LED_OFF};

    @Override
    public void runOpMode()
           throws InterruptedException{
        try {
            // init
            robot = new RobotRevAmpedLinearTest(this);
            drive = robot.getSwerveDriveLinear();
            //autonomous function
            run();

        } catch (Exception e) {
            robot.close1();
        } finally {
            // shut down
            if (robot != null) {
                robot.close();
            }
        }
    }
    /*
        Function used to knock off gold mineral in autonomous
        Total time taken 4 seconds
     */
    private void intakeMineral(boolean isCrater)
        throws InterruptedException{
        try {
            robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_IN);
            int positionSlideStart = robot.motorSlide.getCurrentPosition();
            int positionSlide = positionSlideStart;
            long startTime = System.currentTimeMillis();
            long time = startTime;
            boolean isTouchDown = robot.switchSlideDown.isTouch();
            //bringing slide out
            while (positionSlide - positionSlideStart < (isCrater ? 360 : 1000)
                    && time - startTime < (1200) && opModeIsActive()) {
                robot.motorSlide.setPower(-1);
                positionSlide = robot.motorSlide.getCurrentPosition();
                if (!isTouchDown) {
                    robot.motorLatch.setPower(-1);
                } else {
                    robot.motorLatch.setPower(0);
                    robot.servoTelescopeL.setPower(-0.8f);
                    robot.servoTelescopeR.setPower(0.8f);
                }
                isTouchDown = robot.switchSlideDown.isTouch();
                if (!isCrater) {
                    robot.motorIntake.setPower(0.8f);
                }
                time = System.currentTimeMillis();
                idle();
                if (isStopRequested()) {
                    robot.close1();
                }
            }
            robot.motorLatch.setPower(0);
            boolean isSlideBack = robot.switchSlideIn.isTouch();
            //bringing the slide back in
            startTime = System.currentTimeMillis();
            while (!isSlideBack && System.currentTimeMillis() - startTime < 2000 && opModeIsActive()) {
                if (!isCrater) {
                    robot.motorIntake.setPower(0.8f);
                }
                robot.motorSlide.setPower(1);
                isSlideBack = robot.switchSlideIn.isTouch();
                robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_OUT);
                //if the hang is down, start raising the popper
                if (isTouchDown) {
                    robot.servoTelescopeL.setPower(-0.8f);
                    robot.servoTelescopeR.setPower(0.8f);
                } else {
                    robot.motorLatch.setPower(-1);
                    robot.servoTelescopeL.setPower(0);
                    robot.servoTelescopeR.setPower(0);
                }
                if (isStopRequested()) {
                    robot.close1();
                }
                idle();
            }
            // temp debug 02-02
            robot.motorSlide.setPower(1f);
            sleep(750);
            robot.motorIntake.setPower(0);
            robot.motorSlide.setPower(0);

        } catch (Exception e) {
            robot.close1();
        }
    }
    /*
        Function used to lower latch and raise popper slides, and score gold mineral
        Total Time Taken 6 seconds
     */
    private void scoreMineral()
        throws InterruptedException{
        try {
            boolean isTouchDown = robot.switchSlideDown.isTouch();
            time = System.currentTimeMillis();
            //lower hang
            while (!isTouchDown && opModeIsActive() && !isStopRequested()
                    && System.currentTimeMillis()-time<5000) {
                robot.motorLatch.setPower(-1);
                isTouchDown = robot.switchSlideDown.isTouch();
                idle();
                if (isStopRequested()) {
                    robot.close1();
                }
            }
            robot.motorLatch.setPower(0);
            //raise popper
            long time = System.currentTimeMillis();
            long time1 = System.currentTimeMillis();
            while (opModeIsActive() && !isStopRequested()
                    && time1-time<1000) {
                robot.motorIntake.setPower(0.8f);
                robot.servoTelescopeL.setPower(-0.8f);
                robot.servoTelescopeR.setPower(0.8f);
                time1=System.currentTimeMillis();
                idle();
                if (isStopRequested()) {
                    robot.close1();
                }
            }
            if (opModeIsActive()) {
                robot.motorIntake.setPower(0.8f);
                robot.servoDump.setPosition(RobotRevAmpedConstants.SERVO_DUMP_UP);
                robot.motorPopper.setPower(1);
            }
            robot.servoTelescopeL.setPower(0);
            robot.servoTelescopeR.setPower(0);
            try {
                Thread.sleep(1500);
                robot.motorPopper.setPower(0);
                Thread.sleep(500);
                robot.motorIntake.setPower(0);
                robot.servoDump.setPosition(RobotRevAmpedConstants.SERVO_DUMP);
                Thread.sleep(800);
            } catch (Exception e) {}
            robot.servoDump.setPosition(RobotRevAmpedConstants.SERVO_DUMP_UP);
            robot.motorPopper.setPower(0);
        } catch (Exception e) {
            robot.close1();
        }
    }
    /*
    Function used to dump team marker
    Total Time 4.5 seconds
     */
    private void dumpTM(int order, boolean isCrater) {
        try {
            //get the initial positions/times
            int positionSlideStart = robot.motorSlide.getCurrentPosition();
            int positionSlide = positionSlideStart;
            long startTime = System.currentTimeMillis();
            long time = startTime;
            boolean isTouchDown = robot.switchSlideDown.isTouch();
            if (!isCrater && !isTouchDown && opModeIsActive()) {
                robot.motorLatch.setPower(-1);
            }
            //take the slide out
            while (positionSlide - positionSlideStart < 1800 && time - startTime < 1500 && opModeIsActive()) {
                robot.motorSlide.setPower(-1);
                positionSlide = robot.motorSlide.getCurrentPosition();
                time = System.currentTimeMillis();
                //robot.motorIntake.setPower(0.4f);
                idle();
                if (isStopRequested()) {
                    robot.close1();
                }
            }
            //dump the team mineral
            if (opModeIsActive()) {
                robot.motorIntake.setPower(-1f);
            }
            try {
                Thread.sleep(750);
            } catch (Exception e) {}
            boolean isSlideBack = robot.switchSlideIn.isTouch();

            startTime = System.currentTimeMillis();
            //bring slide back
            while (!isSlideBack && System.currentTimeMillis() - startTime < 2500&& opModeIsActive()) {
                robot.motorIntake.setPower(0);
                robot.motorSlide.setPower(1);
                isSlideBack = robot.switchSlideIn.isTouch();
                robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_OUT);
                if (!isCrater) {
                    if (!isTouchDown) {
                        robot.motorLatch.setPower(-1);
                    } else {
                        robot.motorLatch.setPower(0);
                        robot.servoTelescopeL.setPower(-0.8f);
                        robot.servoTelescopeR.setPower(0.8f);
                    }
                }
                if (isStopRequested()) {
                    robot.close1();
                }
                isTouchDown = robot.switchSlideDown.isTouch();
                idle();
            }
            robot.motorLatch.setPower(0);
            robot.motorSlide.setPower(0);
            robot.motorIntake.setPower(0);
        } catch (Exception e) {
            robot.close1();
        }
    }

    public void run()
    {
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
                boolean isDoubleSample;
                if (!isCrater) {
                    isDoubleSample = false;
                } else {
                    isDoubleSample = sl.selectDoubleSample();
                    telemetry.addData("Double Sampling", isDoubleSample ? "Yes" : "No");
                }
                boolean isRed = sl.selectAlliance();
                boolean isHang = sl.selectHang();
                boolean isSample = sl.selectSample();
                boolean isScore = sl.selectScore();
                if (isCrater) {
                    isScore = false;
                } else if (!isSample) {
                    isScore = false;
                }
                telemetry.addData("Crater", isCrater ? "Yes" : "No");
                telemetry.addData("isRed?", isRed ? "Yes" : "No");
                telemetry.addData("isHang?", isHang ? "Yes" : "No");
                telemetry.addData("isSample?", isSample ? "Yes" : "No");
                telemetry.addData("isScore?", isScore ? "Yes" : "No");
                telemetry.update();
                robot.drawLed();
                if (isRed) {
                    robot.ledRed.set(HwLed.ELedStatus.LED_ON);
                } else {
                    robot.ledBlue.set(HwLed.ELedStatus.LED_ON);
                }
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
                if (isHang) {
                    long time = startTime;
                    boolean isTouchUp = robot.switchSlideUp.isTouch();
                    while (!isTouchUp && time - startTime < 5000 && opModeIsActive()) {
                        robot.motorLatch.setPower(1);
                        time = System.currentTimeMillis();
                        isTouchUp = robot.switchSlideUp.isTouch();
                        idle();
                        if (isStopRequested()) {
                            robot.close1();
                        }
                    }
                    robot.motorLatch.setPower(0);
                }
                //if we are towards the crater
                if (isCrater) {
                    //drive forward
                    drive.moveToEncoderInch(TurnType.FORWARD,
                            4,
                            0.8f*gearFactor,
                            5000,
                            false,
                            false,
                            true);
                    if (isHang && opModeIsActive() && !isStopRequested()) {
                        robot.motorLatch.setPower(-1);
                    }
                    //if the gold is left position
                    if (order == SampleOrderDetector.GoldenOrder.LEFT) {
                        drive.turn2(TurnType.TURN_REGULAR,
                                13,
                                0.6f*gearFactor,
                                3000,
                                false);
                        robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_IN);
                        lp.waitMillis(200);
                        intakeMineral(true);
                        //move into position to park
                        drive.turn2(TurnType.TURN_REGULAR,
                                -13,
                                0.6f*gearFactor,
                                4000,
                                false);
                    } else if (order == SampleOrderDetector.GoldenOrder.CENTER) {
                        robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_IN);
                        lp.waitMillis(200);
                        intakeMineral(true);
                    } else if (order == SampleOrderDetector.GoldenOrder.RIGHT) {
                        drive.turn2(TurnType.TURN_REGULAR,
                                -15,
                                0.6f*gearFactor,
                                4000,
                                false);
                        robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_IN);
                        lp.waitMillis(200);
                        intakeMineral(true);
                        //move into position to park
                        drive.turn2(TurnType.TURN_REGULAR,
                                15,
                                0.6f*gearFactor,
                                4000,
                                false);
                    }
                    if (isStopRequested()) {
                        robot.close1();
                        robot.close();
                    }

                    //move forward to clear the legs of the lander\
                    if (order== SampleOrderDetector.GoldenOrder.LEFT) {
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                9,
                                0.6f*gearFactor,
                                2000,
                                false,
                                false,
                                true);
                    } else {
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                9,
                                0.6f*gearFactor,
                                2000,
                                false,
                                false,
                                true);
                    }
                    //turn and strafe to wall, with intake facing the depot
                    drive.turn2(TurnType.TURN_REGULAR,
                            -178,
                            0.9f*gearFactor,
                            6000,
                            true);
                    //strafe to the wall
                    if (!(order== SampleOrderDetector.GoldenOrder.RIGHT) || !isDoubleSample) {
                        drive.moveToEncoderInch(TurnType.STRAFE,
                                43,
                                0.8f*gearFactor,
                                2000,
                                false,
                                false,
                                true);
                        drive.turn(TurnType.TURN_REGULAR,
                                -34,
                                0.7f*gearFactor,
                                1000,
                                false);
                    } else {
                        drive.moveToEncoderInch(TurnType.STRAFE,
                                30,
                                0.8f*gearFactor,
                                2000,
                                false,
                                false,
                                true);
                        drive.turn(TurnType.TURN_REGULAR,
                                -30,
                                0.7f*gearFactor,
                                1000,
                                false);
                    }
                    //turn to align to the wall
                    if (isStopRequested()) {
                        robot.close1();
                        robot.close();
                    }
                    //strafe to align with wall
                    if (!(order==SampleOrderDetector.GoldenOrder.RIGHT) || !isDoubleSample) {
                        drive.moveToEncoderInch(TurnType.STRAFE,
                                6,
                                0.8f*gearFactor,
                                2000,
                                false,
                                false,
                                true);
                        drive.moveToTime(TurnType.STRAFE, -0.4f, 350);
                    }

                    //dump mineral and park into the crater
                    if (!isDoubleSample) {
                        robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_IN);
                        lp.waitMillis(200);
                        dumpTM(1, true);
                        //drive back to the crater
                        drive.moveToEncoderInch(TurnType.STRAFE,
                                doubleSampleStrafe,
                                0.9f*gearFactor,
                                2000,
                                false,
                                false,
                                true);
                        if (opModeIsActive()) {
                            robot.servoTelescopeL.setPower(-0.8f);
                            robot.servoTelescopeR.setPower(0.8f);
                        }
                        drive.turn2(TurnType.TURN_REGULAR,
                                -145,
                                0.6f*gearFactor,
                                3000,
                                false);
                        if (isStopRequested()) {
                            robot.close1();
                            robot.close();
                        }
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                28,
                                0.8f*gearFactor,
                                5000,
                                false,
                                false,
                                true);
                        robot.servoTelescopeL.setPower(0);
                        robot.servoTelescopeR.setPower(0);
                    } else {
                        if (order==SampleOrderDetector.GoldenOrder.LEFT) {
                            drive.moveToEncoderInch(TurnType.FORWARD,
                                    distanceToDepot-4,
                                    0.9f*gearFactor,
                                    4000,
                                    false,
                                    false,
                                    true)  ;
                            drive.moveToEncoderInch(TurnType.STRAFE,
                                    doubleSampleStrafe-21,
                                    0.9f*gearFactor,
                                    3000,
                                    false,
                                    false,
                                    true);
                            drive.moveToEncoderInch(TurnType.STRAFE,
                                    -doubleSampleStrafe+18,
                                    0.9f*gearFactor,
                                    3000,
                                    false,
                                    false,
                                    true);
                            if (isStopRequested()) {
                                robot.close1();
                                robot.close();
                            }
                            drive.turn2(TurnType.TURN_REGULAR,
                                    -40,
                                    0.7f*gearFactor,
                                    3000,
                                    false);
                            robot.motorIntake.setPower(-0.75f);
                            lp.waitMillis(600);
                            robot.motorIntake.setPower(0f);
                            if (opModeIsActive()) {
                                robot.servoTelescopeL.setPower(-0.8f);
                                robot.servoTelescopeR.setPower(0.8f);
                            }
                            drive.turn2(TurnType.TURN_REGULAR,
                                    -85,
                                    0.7f*gearFactor,
                                    3000,
                                    false);
                            /*drive.moveToEncoderInch(TurnType.STRAFE,
                                    -5,
                                    0.8f,
                                    2000,
                                    false,
                                    false,
                                    true);*/
                        } else if (order==SampleOrderDetector.GoldenOrder.CENTER){
                            drive.moveToEncoderInch(TurnType.FORWARD,
                                    doubleSampleDrive+18,
                                    0.9f*gearFactor,
                                    2000,
                                    false,
                                    false,
                                    true);
                            drive.moveToEncoderInch(TurnType.STRAFE,
                                    doubleSampleStrafe-10,
                                    0.9f*gearFactor,
                                    2500,
                                    false,
                                    false,
                                    true);
                            drive.moveToEncoderInch(TurnType.STRAFE,
                                    -doubleSampleStrafe+12,
                                    0.9f*gearFactor,
                                    2500,
                                    false,
                                    false,
                                    true);
                            if (isStopRequested()) {
                                robot.close1();
                                robot.close();
                            }
                            drive.moveToEncoderInch(TurnType.FORWARD,
                                    distanceToDepot-doubleSampleDrive-21,
                                    0.8f*gearFactor,
                                    2500,
                                    false,
                                    false,
                                    true);
                            robot.motorIntake.setPower(-0.75f);
                            lp.waitMillis(800);
                            robot.motorIntake.setPower(0f);
                            drive.moveToEncoderInch(TurnType.STRAFE,
                                    doubleSampleStrafe+1,
                                    0.9f*gearFactor,
                                    2000,
                                    false,
                                    false,
                                    true);
                            if (opModeIsActive()) {
                                robot.servoTelescopeL.setPower(-0.8f);
                                robot.servoTelescopeR.setPower(0.8f);
                            }
                            drive.turn2(TurnType.TURN_REGULAR,
                                    -140,
                                    0.7f*gearFactor,
                                    3000,
                                    false);
                        } else {
                            drive.moveToEncoderInch(TurnType.FORWARD,
                                    distanceToDepot,
                                    0.9f*gearFactor,
                                    2000,
                                    false,
                                    false,
                                    true);
                            if (isStopRequested()) {
                                robot.close1();
                                robot.close();
                            }
                            drive.turn2(TurnType.TURN_REGULAR,
                                    -40,
                                    0.7f*gearFactor,
                                    3000,
                                    false);
                            robot.motorIntake.setPower(-0.75f);
                            lp.waitMillis(600);
                            robot.motorIntake.setPower(0f);
                            //drive back to the crater
                            if (opModeIsActive()) {
                                robot.servoTelescopeL.setPower(-0.8f);
                                robot.servoTelescopeR.setPower(0.8f);
                            }
                            drive.turn2(TurnType.TURN_REGULAR,
                                    -80,
                                    0.7f*gearFactor,
                                    3000,
                                    false);
                        }
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                43,
                                0.85f*gearFactor,
                                5000,
                                false,
                                false,
                                true);
                        robot.servoTelescopeL.setPower(0);
                        robot.servoTelescopeR.setPower(0);
                    }
                } else {
                    //drive forward and dump the team marker, make sure the slide clears the center  ball/mineral
                    drive.moveToEncoderInch(TurnType.FORWARD,
                            18,
                            0.6f*gearFactor,
                            3000,
                            false,
                            false,
                            true);
                    //dump team marker
                    robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_IN);
                    lp.waitMillis(200);
                    dumpTM(1, false);
                    //check if stop requested
                    if (isStopRequested()) {
                        robot.close1();
                        robot.close();
                    }
                    //drive back to knock off gold mineral
                    drive.moveToEncoderInch(TurnType.FORWARD,
                            -9,
                            0.8f*gearFactor,
                            1000,
                            false,
                            false,
                            true);
                    if (!(order == SampleOrderDetector.GoldenOrder.CENTER)) {
                        //Moving to position to intake the gold mineral
                        if (order == SampleOrderDetector.GoldenOrder.LEFT) {
                            //position robot to intake mineral
                            drive.turn(TurnType.TURN_REGULAR,
                                    24,
                                    0.6f*gearFactor,
                                    4000,
                                    false);
                            robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_IN);
                            lp.waitMillis(200);
                            //intake the gold mineral
                            intakeMineral(false);
                            //move into position to park
                            drive.turn2(TurnType.TURN_REGULAR,
                                    -22,
                                    0.6f*gearFactor,
                                    4000,
                                    false);
                        } else {
                            //position robot to intake mineral, mineral is right
                            time = System.currentTimeMillis();
                            drive.turn2(TurnType.TURN_REGULAR,
                                    -23,
                                    0.6f*gearFactor,
                                    3000,
                                    false);
                            //intake the gold mineral
                            robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_IN);
                            lp.waitMillis(200);
                            intakeMineral(false);
                            drive.turn2(TurnType.TURN_REGULAR,
                                    21,
                                    0.6f*gearFactor,
                                    4000,
                                    false);
                        }
                    } else {
                        //gold mineral is center, intake gold mineral
                        robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_IN);
                        lp.waitMillis(200);
                        intakeMineral(false);
                    }
                    //check if stop requested
                    if (isStopRequested()) {
                        robot.close1();
                        robot.close();
                    }
                    if (isScore) {
                        drive.moveToEncoderInch(TurnType.STRAFE,
                                5,
                                0.4f*gearFactor,
                                2000,
                                false,
                                false,
                                true);
                        //score gold mineral
                        scoreMineral();
                    }
                    //drive to park in the crater
                    //clear the legs of the lander
                    drive.moveToEncoderInch(TurnType.FORWARD,
                            5,
                            0.8f*gearFactor,
                            2000,
                            false,
                            false,
                            true);
                    //check if stop requested
                    if (isStopRequested()) {
                        robot.close1();
                        robot.close();
                    }
                    //strafe to the crater
                    if (!(order == SampleOrderDetector.GoldenOrder.LEFT)) {

                            drive.moveToEncoderInch(TurnType.STRAFE,
                                    isScore ? -45 : -40,
                                    0.85f*gearFactor,
                                    3000,
                                    false,
                                    false,
                                    true);

                        //turn to make sure the robot won't hit the wall first
                        //check if stop requested
                        if (isStopRequested()) {
                            robot.close1();
                            robot.close();
                        }
                        drive.turn2(TurnType.TURN_REGULAR,
                                115,
                                0.9f*gearFactor,
                                3000,
                                true);
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                20,
                                0.8f*gearFactor,
                                3000,
                                false,
                                false,
                                true);
                    } else {
                        drive.moveToEncoderInch(TurnType.STRAFE,
                                isScore?-40: -34,
                                0.8f*gearFactor,
                                3000,
                                false,
                                false,
                                true);
                        //check if stop requested
                        if (isStopRequested()) {
                            robot.close1();
                            robot.close();
                        }
                        //turn to make sure the robot won't hit the wall first
                        drive.turn2(TurnType.TURN_REGULAR,
                                120,
                                0.9f*gearFactor,
                                3000,
                                true);
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                16,
                                0.8f*gearFactor,
                                3000,
                                false,
                                false,
                                true);
                    }

                }
                telemetry.addLine("Six Glyph Auto Done!!!!!");
                telemetry.update();
                RobotLog.i("AutonomousRevamped finish in " + (System.currentTimeMillis() - startTime) + " milliseconds");
                lp.waitMillis(100);
            }
        } catch (Exception e) {
            robot.close1();
        }
    }

}
