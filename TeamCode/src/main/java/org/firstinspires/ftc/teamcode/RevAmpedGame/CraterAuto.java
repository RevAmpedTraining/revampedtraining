package org.firstinspires.ftc.teamcode.RevAmpedGame;

import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.QuinticSplineSegment;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.revAmped.components.Drive;
import com.revAmped.components.HwDevice;
import com.revAmped.components.TurnType;
import com.revAmped.config.RobotRevAmpedConstants;
import com.revAmped.config.SwerveDriveConstants;
import com.revAmped.linear.components.RobotRevAmpedLinearTest;
import com.revAmped.linear.components.SwerveDriveLinear;
import com.revAmped.linear.util.SelectLinear;
import com.revAmped.linear.util.WaitLinear;
import com.revAmped.util.SampleOrderDetector;

import android.drm.DrmStore;
import android.graphics.Bitmap;
import android.provider.Settings;

import com.revAmped.sensors.VuMarkSensing;
import com.revAmped.util.Stalled;
import com.vuforia.INIT_ERRORCODE;

import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

/**
 * Created by John Wang on 10/27/2018.
 */
//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Crater Autonomous", group="Game")
public class CraterAuto
        extends LinearOpMode {
    //initialize the robot objects
    private RobotRevAmpedLinearTest robot;

    private SwerveDriveLinear drive;
    //acceleration/decceleration constants
    float POWER_START = 0.17f;

    float POWER_STOP = 0.10f;
    //top hang position
    int midHangPos = 5700;
    //popper reset constants
    int newTarget;
    int count_per_turn = 1120; // this is Count per turn
    double degree_target_abs = 359; // this is Abs dress target  final position shouls be inside 300-359.99
    double count_current = 0; // this is Abs degree target + N* turn
    double count_to_go = 0; //
    double temp = 3%1;

    private final int doubleSampleDriveC = 45;

    private final int doubleSampleStrafeC = 18;

    private final int doubleSampleStrafeL = 26;
    //how far away from depot in crater position
    private final int distanceToDepot = 56;
    //distance to crater
    private final int distanceToCraterCrab = 32;

    private final int distanceToCrater = 26;

    @Override
    public void runOpMode()
            throws InterruptedException{
        try {
            // init
            robot = new RobotRevAmpedLinearTest(this);
            drive = robot.getSwerveDriveLinear();
            //autonomous function
            run();

        } catch (InterruptedException e) {
            robot.close1();
            robot.close();
        } finally {
            // shut down
            if (robot != null) {
                robot.close();
            }
        }
    }

    private void moveToEncoderCrater(TurnType dir,
                                     int tick,
                                     float maxPower,
                                     int timeoutMillis,
                                     boolean isAccelerate,
                                     boolean isBreak,
                                     boolean lowerLatch,
                                     boolean pop,
                                     boolean popSlides,
                                     boolean raiseLatch,
                                     boolean extendo)
            throws InterruptedException
    {
        if (pop) {
            robot.motorIntake.setPower(1);
            robot.motorPopper.setPower(1);
        }
        tick = drive.inchToTick(tick);
        final float MOVE_TIMEOUT_FACTOR = 2f;
        final int MOVE_STALL_TIME = 500;

        maxPower = Range.clip(Math.abs(maxPower), 0f, 1f);

        if (timeoutMillis < 0) {
            timeoutMillis = Math.abs((int) (tick * Drive.AM20_MILLISECOND_PER_TICK * MOVE_TIMEOUT_FACTOR)) + 2000;
        }

        // turn wheels
        if ((dir!=TurnType.CURVE) && (dir!=TurnType.STRAFE_LEFT_DIAG) && (dir!=TurnType.STRAFE_RIGHT_DIAG)) {
            drive.setTurnWait(dir);
        } else if (dir==TurnType.CURVE){
            robot.servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_CRABLEFT+25/255f);
            robot.servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_CRABLEFT+25/255f);

            robot.servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_CRABLEFT+25/255f);
            robot.servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_CRABLEFT+25/255f);

        } else if (dir == TurnType.STRAFE_LEFT_DIAG) {
            robot.servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_CRABLEFT+15/255f);
            robot.servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_CRABLEFT+15/255f);

            robot.servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_CRABLEFT+5/255f);
            robot.servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_CRABLEFT+5/255f);
        } else {
            robot.servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_CRABRIGHT-8/255f);
            robot.servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_CRABRIGHT-8/255f);

            robot.servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_CRABRIGHT-20/255f);
            robot.servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_CRABRIGHT-20/255f);
        }

        drive.resetPosition();
        // is RUN_TO_POSITION
        drive.setMode2(DcMotor.RunMode.RUN_TO_POSITION);
        if (dir == TurnType.STRAFE || dir==TurnType.FORWARD_LEFT) {
            drive.setTargetPosition2(tick,
                    new HwDevice[]{drive.driveRightBack, drive.driveLeftFront});
            drive.setTargetPosition2(-tick,
                    new HwDevice[]{drive.driveRightFront, drive.driveLeftBack});
        } else if (dir ==TurnType.TURN_SWERVE_FWD_TURN){
            drive.setTargetPosition2(tick,
                    new HwDevice[]{drive.driveRightBack, drive.driveLeftBack});
            drive.setTargetPosition2(-tick,
                    new HwDevice[]{drive.driveRightFront, drive.driveLeftFront});
        } else if (dir == TurnType.CURVE) {
            drive.setTargetPosition2(tick);
        } else if (dir == TurnType.LEFT_CURVE || dir==TurnType.LEFT_CURVE_1) {
            drive.setTargetPosition2(tick,
                    new HwDevice[]{drive.driveRightBack, drive.driveLeftBack, drive.driveLeftFront});
            drive.setTargetPosition2(-tick,
                    new HwDevice[]{drive.driveRightFront});
        } else {
            drive.setTargetPosition2(-tick);

        }

        long startTimestamp = System.currentTimeMillis();
        long timeStamp = startTimestamp;
        Stalled stalled = new Stalled();
        int startAverage = ((int)Math.signum(tick))*drive.getEncoder(dir);
        int startDelta = tick - startAverage;
        while (!isStopRequested() && opModeIsActive() &&
                // is RUN_TO_POSITION
                drive.isBusy() &&
                (timeStamp - startTimestamp < timeoutMillis)) {

            int average = ((int)Math.signum(tick))*drive.getEncoder(dir);
            int delta = tick - average;
            if (Math.abs(delta) < 50)
            {
                // done
                robot.motorIntake.setPower(0);
                robot.motorPopper.setPower(0);
                popperSlide(0);
                robot.motorLatch.setPower(0);
                robot.motorSlide.setPower(0);
                break;
            }
            if (dir==TurnType.CURVE) {
                if (Math.abs(delta) < Math.abs((startDelta / 1.22f))) {
                    drive.setTargetPosition2(tick,
                            new HwDevice[]{drive.driveRightBack, drive.driveLeftBack, drive.driveLeftFront});
                    drive.setTargetPosition2(-tick,
                            new HwDevice[]{drive.driveRightFront});
                    robot.servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_STRAFE_CURVE);
                    robot.servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_STRAFE_CURVE);
                    robot.servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_END);
                    robot.servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_END);
                    maxPower = 0.55f;
                }
            } else if (dir==TurnType.FORWARD_LEFT) {
                if (Math.abs(delta) < Math.abs((startDelta / 2))) {
                    drive.setTargetPosition2(-tick);
                    robot.servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_CRABRIGHT);
                    robot.servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_CRABRIGHT);

                    robot.servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_CRABRIGHT);
                    robot.servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_CRABRIGHT);
                    maxPower = 0.65f;
                }
            }
            float basePower;
            if (!isAccelerate) {
                basePower = Math.signum(delta) * maxPower;
            }
            else {
                if (Math.abs(average) < Math.abs(tick / 1.33)) {
                    // ramp up for 1/3 of distance
                    // minimum power is START_POWER
                    // base is 1600
                    basePower = Math.signum(delta) * maxPower;
                    //basePower = (average/1600f) + Math.signum(tick) * POWER_START;
                } else {
                    // ramp down for 2/3 of distance
                    // minimum power is START_POWER
                    // base is 3200
                    basePower = (delta/4500f) + Math.signum(delta) * POWER_STOP;
                }
                basePower = Range.clip(basePower, -maxPower, maxPower);

            } // if

            //op.telemetry.addData("power", Float.toString(basePower));
            //op.telemetry.addData("travel", Integer.toString(tickToInch(average)));
            //op.telemetry.addData("delta", Integer.toString(tickToInch(delta)));

            if (stalled.isStalled(average, timeStamp)) {
                WaitLinear w = new WaitLinear(this);
                drive.stop2();
                w.waitMillis(MOVE_STALL_TIME);
                timeoutMillis += MOVE_STALL_TIME;
                stalled.reset();
            }
            else {
                if (dir==TurnType.CURVE) {
                    //AREA MAY NEED CHANGE
                    //AREA MAY NEED CHANGE - decreasing back power because it will cause too much curvature
                    drive.setPowerCurve(basePower+0.08f/*12f*/, basePower);
                } else if (dir == TurnType.FORWARD_TM) {
                    drive.setPower(basePower,
                            basePower,
                            dir);
                } else {
                    drive.setPower(basePower,
                            basePower,
                            dir);
                }
            } // else
            if (lowerLatch) {
                boolean isSwitchDown = robot.switchSlideDown.isTouch();
                if (!isSwitchDown && System.currentTimeMillis()-startTimestamp>200) {
                    robot.motorLatch.setPower(-1);
                } else {
                    robot.motorLatch.setPower(0);
                }
            }
            if (pop) {
                robot.motorIntake.setPower(1);
                robot.motorPopper.setPower(1);
            }
            if (popSlides) popperSlide(0.85f);
            if (raiseLatch) {
                int positionHang = robot.motorLatch.getCurrentPosition();
                if (positionHang < 6000) {
                    robot.motorLatch.setPower(1);
                } else {
                    robot.motorLatch.setPower(0);
                }
            }
            if (extendo) robot.motorSlide.setPower(1);
            //op.telemetry.update();
            idle();
            timeStamp = System.currentTimeMillis();
        } // while

        if (isBreak) {
            drive.stop2();
        }

        robot.motorIntake.setPower(0);
        robot.motorPopper.setPower(0);
        popperSlide(0);
        robot.motorLatch.setPower(0);
        robot.motorSlide.setPower(0);

        if (Thread.currentThread().isInterrupted()) {
            RobotLog.e("RevAmped Thread interrupted");
        }
        else if (isStopRequested()) {
            RobotLog.e("RevAmped Stop requested");
        }

        // is RUN_TO_POSITION
        drive.setMode2(DcMotor.RunMode.RUN_USING_ENCODER);

        int inches = drive.tickToInch(drive.getEncoder(dir));
        RobotLog.i("Moved " + inches + " inches");
        RobotLog.i("Move took " + (System.currentTimeMillis() - startTimestamp) + " milliseconds");
        //op.telemetry.addData("travel", inches);
        idle();
    }

    private void popperSlide(float p)
        throws InterruptedException {
        robot.servoTelescopeR.setPower(p);
        robot.servoTelescopeL.setPower(-p);
    }

    private void lockServos ()
        throws InterruptedException {
        robot.servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_END);
        robot.servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_END);

        robot.servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_START);
        robot.servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_START);
    }
    private void unlockServos ()
        throws InterruptedException {
        robot.servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_START);
        robot.servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_START);

        robot.servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_START);
        robot.servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_START);
    }



    /*
        Function used to knock off gold mineral in autonomous
        Total time taken 3 seconds
     */
    private void intakeMineral(boolean lowerLatch, int i, int intakeTime, int intakePosition)
            throws InterruptedException{
        try {
            int positionSlideStart = robot.motorSlide.getCurrentPosition();
            long startTime = System.currentTimeMillis();
            boolean isSwitchDown = robot.switchSlideDown.isTouch();
            //bringing slide out
            while (robot.motorSlide.getCurrentPosition() - positionSlideStart < (intakePosition)
                    && System.currentTimeMillis() - startTime < (intakeTime) && opModeIsActive() && !isStopRequested()) {
                robot.motorSlide.setPower(-1);
                robot.motorIntake.setPower(1);
                if (lowerLatch && !isSwitchDown) robot.motorLatch.setPower(-1);
                isSwitchDown = robot.switchSlideDown.isTouch();
                //encoder reset
                //SAI SUGGESTION
                //lockServos();
                robot.motorPopper.setTargetPosition(newTarget);
                // Turn On RUN_TO_POSITION
                robot.motorPopper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // reset the timeout time and start motion.
                if (opModeIsActive() &&
                        (robot.motorPopper.isBusy() )) {
                    telemetry.addData("count2go", "begining Starting at %7f ", count_to_go);
                    telemetry.update();
                    robot.motorPopper.setPower(-0.75f); // if this number is too big could oscillatiion
                } else {
                    robot.motorPopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);   // back to normal mode
                    robot.motorPopper.setPower(0);
                }
                idle();
                if (isStopRequested()) {
                    robot.close1();
                    break;
                }
            }
            robot.motorPopper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);   // back to normal mode
            robot.motorPopper.setPower(0);
            boolean isSlideBack = robot.switchSlideIn.isTouch();
            //bringing the slide back in
            startTime = System.currentTimeMillis();
            while (!isSlideBack && System.currentTimeMillis() - startTime < 1800 && opModeIsActive() && !isStopRequested()) {
                robot.motorSlide.setPower(1);
                isSlideBack = robot.switchSlideIn.isTouch();
                isSwitchDown = robot.switchSlideDown.isTouch();
                robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_OUT);
                robot.motorIntake.setPower(1);
                //SAI SUGGESTION
                //unlockServos();
                //if the hang is down, start raising the popper
                if (lowerLatch) {
                    popperSlide(0.85f);
                    if (!isSwitchDown) robot.motorLatch.setPower(-1);
                    else robot.motorLatch.setPower(0);
                }
                if (isStopRequested()) {
                    robot.close1();
                    break;
                }
                idle();
            }
            popperSlide(0);
            robot.motorLatch.setPower(0);
            robot.motorSlide.setPower(0);
            robot.motorIntake.setPower(0);
        } catch (InterruptedException e) {
            robot.close1();
            robot.close();
        }
    }

    public void run()
        throws InterruptedException{
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
                boolean isCenter = false;
                boolean isLeft = false;
                boolean isScore = false;
                SelectLinear sl = new SelectLinear(this);
                boolean isHang = sl.selectHang();
                boolean isSample = sl.selectSample();
                boolean isDoubleSample = sl.selectDoubleSample();
                boolean scoreTM = sl.selectTM();
                int scoreCycles = sl.adjust("Number of Scoring Cycles", isDoubleSample? 2: scoreTM? 2:3);
                if (scoreCycles != 0 ) isScore = true;
                if (!isHang) scoreCycles=0;
                if (!isSample) isDoubleSample = false;
                WaitLinear lp = new WaitLinear(this);
                robot.drawLed();
                robot.ledGreen.on();
                robot.gyroSensor.resetHeading();
                robot.motorLatch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //stall until play is pressed
                while (!opModeIsActive() && !isStopRequested()) {
                    telemetry.addData("status", "waiting for start command...");
                    telemetry.addData("Score Team Marker?", scoreTM);
                    telemetry.addData("#Score Cycles?", scoreCycles);
                    telemetry.addData("isHang?", isHang ? "Yes" : "No");
                    telemetry.addData("isSample?", isSample ? "Yes" : "No");
                    telemetry.addData("isDoubleSample?", isDoubleSample ? "Yes" : "No");
                    telemetry.update();
                }
                long startTimeStamp = System.currentTimeMillis();
                robot.ledRed.off();
                robot.ledBlue.off();
                robot.ledGreen.off();
                robot.ledWhite.off();
                robot.ledYellow.off();
                robot.drawLed();
                //Landing
                long startTime = System.currentTimeMillis();
                robot.motorLatch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                boolean sampled = false;
                SampleOrderDetector.GoldenOrder order = SampleOrderDetector.GoldenOrder.UNKNOWN;
                if (isHang) {
                    boolean isTouchUp = robot.switchSlideUp.isTouch();
                    while (!isTouchUp && System.currentTimeMillis() - startTime < 5250
                            && opModeIsActive() && !isStopRequested()) {
                        robot.motorLatch.setPower(1);
                        isTouchUp = robot.switchSlideUp.isTouch();
                        idle();
                        if (isStopRequested()) robot.close1();
                        //sample during landing for saving time
                        if (isSample && !sampled) {
                            long startAnalyzeTime = System.currentTimeMillis();
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
                            if (order == SampleOrderDetector.GoldenOrder.UNKNOWN) order = SampleOrderDetector.GoldenOrder.RIGHT;
                            long endAnalyzeTime = System.currentTimeMillis();
                            telemetry.addData("Gold", order);
                            telemetry.addData("Time to Analyze", endAnalyzeTime-startAnalyzeTime);
                            telemetry.update();
                            sampled = true;
                        }
                        if (order== SampleOrderDetector.GoldenOrder.CENTER) isCenter = true;
                        else if (order== SampleOrderDetector.GoldenOrder.LEFT) isLeft = true;
                    }
                    robot.motorLatch.setPower(0);
                    popperSlide(0);
                } else if (isSample) {
                        long startAnalyzeTime = System.currentTimeMillis();
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
                        if (order == SampleOrderDetector.GoldenOrder.UNKNOWN) order = SampleOrderDetector.GoldenOrder.RIGHT;
                        telemetry.addData("Gold", order);
                        long endAnalyzeTime = System.currentTimeMillis();
                        telemetry.addData("Time to Analyze", endAnalyzeTime-startAnalyzeTime);
                        telemetry.update();
                    if (order == SampleOrderDetector.GoldenOrder.CENTER) isCenter = true;
                    else if (order == SampleOrderDetector.GoldenOrder.LEFT) isLeft = true;
                }
                if (order != SampleOrderDetector.GoldenOrder.CENTER && scoreCycles==2) {
                    scoreCycles = 1;
                }
                popperSlide(0);
                if (scoreCycles != 0) {
                    for (int i = 0; i < scoreCycles; i++) {
                        startTime = System.currentTimeMillis();
                        if ((startTime - startTimeStamp) < 16000) {
                            TurnType MINERAL_POSITION = TurnType.FORWARD;
                            isCenter = true;
                            if (order == SampleOrderDetector.GoldenOrder.LEFT) {
                                MINERAL_POSITION = TurnType.STRAFE_LEFT_DIAG;
                                isCenter = false;
                                if (i!=0) MINERAL_POSITION = TurnType.DIAG_LEFT;
                            } else if (order == SampleOrderDetector.GoldenOrder.CENTER) {
                                MINERAL_POSITION = TurnType.FORWARD;
                                if (i!=0) MINERAL_POSITION = TurnType.FORWARD_RIGHT;
                            } else if (order == SampleOrderDetector.GoldenOrder.RIGHT) {
                                MINERAL_POSITION = TurnType.STRAFE_RIGHT_DIAG;
                                isCenter = false;

                            }
                            boolean raiseLatch = false;
                            boolean raisePop = false;
                            if (i!=0) {
                                raiseLatch = true;
                                raisePop = true;
                            }

                                    moveToEncoderCrater(/*isLeft?TurnType.DIAG_LEFT :*/ MINERAL_POSITION,
                                        isCenter ? distanceToCrater : isLeft? distanceToCraterCrab:distanceToCraterCrab+1,
                                        0.65f,
                                        2000,
                                        false,
                                        true,
                                        true,
                                        false,
                                        raisePop,
                                        raiseLatch,
                                        false);
                            robot.servoDump.setPosition(RobotRevAmpedConstants.SERVO_DUMP_UP);
                            robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_IN);
                            //lp.waitMillis(100);
                            count_current = robot.motorPopper.getCurrentPosition() % count_per_turn; // convert to in one cycle ie 370->10
                            //count_to_go = (count_per_turn) - count_current;
                            count_to_go = count_current;
                            telemetry.addData("current", "Starting at %7f ", count_current);
                            telemetry.addData("count2go", "Starting at %7f ", count_to_go);
                            if (count_to_go > 0) {
                                newTarget = robot.motorPopper.getCurrentPosition() - (int) (count_to_go) + 30; // add 30 count ~ 1 degree, make sure 2nd time alway pass.....
                                telemetry.addData("target GO ", "Starting at %7d ", newTarget);
                            }
                            if (i == 0){
                                intakeMineral(true, i, 1350, isCenter? 420:300);
                                robot.servoDump.setPosition(RobotRevAmpedConstants.SERVO_DUMP_UP);
                                moveToEncoderCrater(MINERAL_POSITION,
                                        (isCenter? 8 : isLeft? 8 : 9)-(isCenter ? distanceToCrater : distanceToCraterCrab),
                                        0.65f,
                                        2000,
                                        false,
                                        true,
                                        true,
                                        true,
                                        true,
                                        false,
                                        false);
                                long slideTime = System.currentTimeMillis();
                                while (System.currentTimeMillis()-slideTime<(isCenter?2500: isLeft?2500 : 2500)
                                        && opModeIsActive() && !isStopRequested()) {
                                    popperSlide(0.85f);
                                    robot.motorPopper.setPower(1);
                                    robot.motorIntake.setPower(1);
                                }
                                //AREA MAY NEED CHANGE - right may also need change between strafing or curving
                                //AREA MAY NEED CHANGE - added back instead of crab right for left
                                // 4/15 at pdx to prevent catching, may need changing
                                if (order== SampleOrderDetector.GoldenOrder.RIGHT) {
                                    if (opModeIsActive() && !isStopRequested()) {
                                        popperSlide(0.85f);
                                        robot.motorPopper.setPower(1);
                                        robot.motorIntake.setPower(1);
                                    }
                                }
                                    moveToEncoderCrater(isCenter? TurnType.STRAFE_RIGHT_DIAG :
                                                    isLeft? TurnType.STRAFE_RIGHT_DIAG : /*TurnType.LEFT_CURVE*/TurnType.STRAFE_RIGHT_DIAG,
                                            isCenter?-8:isLeft?-4:-4/*-5*/,
                                            0.65f,
                                            2000,
                                            false,
                                            true,
                                            false,
                                            true,
                                            true,
                                            false,
                                            false);
                               /*if (order== SampleOrderDetector.GoldenOrder.RIGHT) {
                                    moveToEncoderCrater(TurnType.FORWARD,
                                            -6,
                                            0.6f,
                                            2000,
                                            false,
                                            true,
                                            false,
                                            true,
                                            true,
                                            false,
                                            false);
                                }*/
                                if (opModeIsActive() && !isStopRequested()) {
                                    popperSlide(0.85f);
                                    robot.motorPopper.setPower(1);
                                    robot.motorIntake.setPower(1);
                                }
                            } else {
                                long startSlideTime = System.currentTimeMillis();
                                intakeMineral(false, i, isLeft? 1400:1500, isCenter? 420 : 350);
                                //changed to all positions to give the robot a little time to pop 4/16
                                while (System.currentTimeMillis()-startSlideTime<(isCenter?500:isLeft? 600 : 400)
                                        && opModeIsActive() && !isStopRequested()) {
                                    robot.motorPopper.setPower(1);
                                    robot.motorIntake.setPower(1);
                                }
                                //give slightly more to right the second time to get to the crater
                                //commented out 4/17 to test
                                if (order== SampleOrderDetector.GoldenOrder.LEFT) MINERAL_POSITION = TurnType.STRAFE_LEFT_DIAG;
                                moveToEncoderCrater(isLeft? TurnType.DIAG_LEFT : MINERAL_POSITION,
                                        -(isCenter ? distanceToCrater :
                                                isLeft? distanceToCraterCrab : distanceToCraterCrab + 4),
                                        0.65f,
                                        2000,
                                        false,
                                        true,
                                        false,
                                        true,
                                        false,
                                        true,
                                        false);
                                if (opModeIsActive() && !isStopRequested()) {
                                    popperSlide(0.85f);
                                    robot.motorPopper.setPower(1);
                                    robot.motorIntake.setPower(1);
                                }
                                lp.waitMillis(400);
                            }
                            robot.servoDump.setPosition(RobotRevAmpedConstants.SERVO_DUMP);
                            if (opModeIsActive() && !isStopRequested()) {
                                lp.waitMillis(700);
                            }
                            popperSlide(0);
                            robot.motorPopper.setPower(0);
                            robot.motorIntake.setPower(0);
                        } else {
                            break;
                        }
                    }
                    moveToEncoderCrater(TurnType.CURVE,
                            isLeft?-51:-50,
                            //isCenter?-45:isLeft? -45:-45,
                            0.65f,
                            4000,
                            false,
                            true,
                            false,
                            false,
                            false,
                            true,
                            false);
                    robot.servoDump.setPosition(RobotRevAmpedConstants.SERVO_DUMP_UP);
                } else {
                    TurnType MINERAL_POSITION = TurnType.STRAFE_RIGHT_DIAG;
                    isCenter = false;
                    if (order == SampleOrderDetector.GoldenOrder.LEFT) {
                        MINERAL_POSITION = TurnType.STRAFE_LEFT_DIAG;
                    } else if (order== SampleOrderDetector.GoldenOrder.CENTER) {
                        MINERAL_POSITION = TurnType.FORWARD;
                        isCenter = true;
                    }
                    moveToEncoderCrater(MINERAL_POSITION,
                            isCenter? distanceToCrater : distanceToCraterCrab,
                            0.6f,
                            4000,
                            true,
                            true,
                            true,
                            false,
                            false,
                            false,
                            false);
                    long latchTime = System.currentTimeMillis();
                    boolean isSwitchDown = robot.switchSlideDown.isTouch();
                    while (!isSwitchDown&& System.currentTimeMillis()-latchTime<2500 && opModeIsActive() &&
                            !isStopRequested()) {
                        robot.motorLatch.setPower(-1);
                        isSwitchDown = robot.switchSlideDown.isTouch();
                    }
                    robot.motorLatch.setPower(0);
                    popperSlide(0);
                    if (order== SampleOrderDetector.GoldenOrder.LEFT) {
                        moveToEncoderCrater(TurnType.FORWARD,
                                -11,
                                0.5f,
                                4000,
                                false,
                                true,
                                true,
                                false,
                                true,
                                false,
                                false);
                        moveToEncoderCrater(TurnType.LEFT_CURVE,
                                -32,
                                0.5f,
                                3000,
                                false,
                                true,
                                true,
                                false,
                                true,
                                false,
                                false);
                    } else {
                        moveToEncoderCrater(MINERAL_POSITION ,
                                (isCenter?8:8) - (isCenter ? distanceToCrater : distanceToCraterCrab),
                                0.5f,
                                4000,
                                false,
                                true,
                                true,
                                false,
                                true,
                                false,
                                false);
                        moveToEncoderCrater(isCenter?TurnType.STRAFE_RIGHT_DIAG: TurnType.LEFT_CURVE,
                                isCenter?-8:-6,
                                0.5f,
                                2000,
                                false,
                                true,
                                false,
                                false,
                                true,
                                false,
                                false);
                        if (order== SampleOrderDetector.GoldenOrder.RIGHT) {
                            moveToEncoderCrater(TurnType.FORWARD,
                                    -7,
                                    0.5f,
                                    2000,
                                    false,
                                    true,
                                    false,
                                    false,
                                    true,
                                    false,
                                    false);
                        }
                        moveToEncoderCrater(TurnType.CURVE,
                                isCenter?-45:-45,
                                0.65f,
                                4000,
                                false,
                                true,
                                false,
                                false,
                                true,
                                true,
                                false);
                    }
                }
                boolean popSlides = false;
                if (scoreCycles==0) popSlides = true;
                lp.waitMillis(100);
                    if (scoreTM) {
                        if (!isDoubleSample) {
                            moveToEncoderCrater(TurnType.FORWARD_TM,
                                    -54,
                                    0.8f,
                                    5000,
                                    true,
                                    true,
                                    false,
                                    false,
                                    popSlides,
                                    true,
                                    false);
                            //team marker dump
                            robot.servoTeam.setPosition(RobotRevAmpedConstants.SERVO_MARKER_DUMP);
                            moveToEncoderCrater(TurnType.FORWARD,
                                    64,
                                    0.92f,
                                    5000,
                                    true,
                                    true,
                                    false,
                                    false,
                                    popSlides,
                                    true,
                                    false);
                            robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_IN);
                            long endingraisePopper = System.currentTimeMillis();
                            while (scoreCycles==0 && System.currentTimeMillis()-endingraisePopper < 3000
                                    && !isStopRequested() && opModeIsActive()) {
                                popperSlide(0.85f);
                                if (System.currentTimeMillis()-endingraisePopper > 1000 &&
                                        robot.motorLatch.getCurrentPosition() <6000) {
                                    robot.motorLatch.setPower(1);
                                }
                            }
                            robot.motorLatch.setPower(0);
                            popperSlide(0);

                        } else {
                            if (order != SampleOrderDetector.GoldenOrder.RIGHT) {
                                moveToEncoderCrater(TurnType.FORWARD_TM,
                                        isLeft ? -distanceToDepot : -doubleSampleDriveC,
                                        0.7f,
                                        4000,
                                        false,
                                        true,
                                        false,
                                        false,
                                        popSlides,
                                        true,
                                        false);
                                moveToEncoderCrater(TurnType.STRAFE,
                                        isLeft ? doubleSampleStrafeL : doubleSampleStrafeC ,
                                        0.7f,
                                        3000,
                                        false,
                                        false,
                                        false,
                                        false,
                                        false,
                                        true,
                                        false);
                                moveToEncoderCrater(TurnType.STRAFE,
                                        isLeft ? -doubleSampleStrafeL - 3:
                                                -doubleSampleStrafeC - 3,
                                        0.6f,
                                        3000,
                                        false,
                                        true,
                                        false,
                                        false,
                                        popSlides,
                                        true,
                                        false);
                                //robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_IN);
                                //team marker dump
                                robot.servoTeam.setPosition(RobotRevAmpedConstants.SERVO_MARKER_DUMP);
                                moveToEncoderCrater(TurnType.FORWARD,
                                        isLeft? 64 : /*54 was b4, may change 4/11*/50,
                                        0.9f,
                                        5000,
                                        true,
                                        true,
                                        false,
                                        false,
                                        false,
                                        true,
                                        false);
                            } else {
                                moveToEncoderCrater(TurnType.FORWARD_TM,
                                        -60,
                                        0.8f,
                                        5000,
                                        true,
                                        true,
                                        false,
                                        false,
                                        false,
                                        true,
                                        false);
                                robot.servoTeam.setPosition(RobotRevAmpedConstants.SERVO_MARKER_DUMP);
                                moveToEncoderCrater(TurnType.FORWARD_RIGHT_DS,
                                        40,
                                        0.7f,
                                        3000,
                                        false,
                                        true,
                                        false,
                                        false,
                                        false,
                                        false,
                                        false);
                                moveToEncoderCrater(TurnType.DIAG_LEFT,
                                        28,
                                        0.7f,
                                        3000,
                                        true,
                                        true,
                                        false,
                                        false,
                                        false,
                                        false,
                                        false);
                            }
                        }
                    } else {
                        moveToEncoderCrater(TurnType.FORWARD,
                                20,
                                0.4f,
                                3000,
                                false,
                                true,
                                true,
                                false,
                                true,
                                false,
                                false);

                        if (System.currentTimeMillis()-startTimeStamp < 25000) {
                            startTime = System.currentTimeMillis();
                            while (opModeIsActive() && !isStopRequested() &&
                                    System.currentTimeMillis()-startTime < 4000) {
                                popperSlide(0.85f);
                                int positionHang = robot.motorLatch.getCurrentPosition();
                                if (positionHang < midHangPos) {
                                    robot.motorLatch.setPower(1);
                                } else {
                                    robot.motorLatch.setPower(0);
                                }
                            }
                        }
                        popperSlide(0);
                    }
                telemetry.addLine("Six Glyph Auto Done!!!!!");
                telemetry.update();
                RobotLog.i("AutonomousRevamped finish in " + (System.currentTimeMillis() - startTime) + " milliseconds");
                lp.waitMillis(100);
            }
        } catch (InterruptedException e) {
            robot.close1();
            robot.close();
        } finally {
            robot.close1();
            robot.close();
        }
    }
}
