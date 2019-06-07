package org.firstinspires.ftc.teamcode.RevAmpedGame;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.revAmped.components.Drive;
import com.revAmped.components.HwDevice;
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
import com.revAmped.util.Stalled;

import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

/**
 * Created by John Wang on 10/27/2018.
 */
//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Depot Autonomous", group="Game")
public class DepotAuto
        extends LinearOpMode {
    //initialize the robot objects

    private RobotRevAmpedLinearTest robot;

    private SwerveDriveLinear drive;

    float POWER_START = 0.17f;

    float POWER_STOP = 0.10f;

    int midHangPos = 5700;


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

    private void moveToEncoderDepot(TurnType dir,
                                     int tick,
                                     float maxPower,
                                     int timeoutMillis,
                                     boolean isAccelerate,
                                     boolean isBreak,
                                     boolean lowerLatch,
                                     boolean pop,
                                     boolean popSlides,
                                    boolean raiseLatch)
            throws InterruptedException
    {
        tick = drive.inchToTick(tick);
        final float MOVE_TIMEOUT_FACTOR = 2f;
        final int MOVE_STALL_TIME = 500;

        maxPower = Math.abs(maxPower);
        maxPower = Range.clip(maxPower, 0f, 1f);

        if (timeoutMillis < 0) {
            timeoutMillis = Math.abs((int) (tick * Drive.AM20_MILLISECOND_PER_TICK * MOVE_TIMEOUT_FACTOR)) + 2000;
        }

        // turn wheels
        drive.setTurnWait(dir);

        drive.resetPosition();
        // is RUN_TO_POSITION
        drive.setMode2(DcMotor.RunMode.RUN_TO_POSITION);
        if (dir == TurnType.STRAFE) {
            drive.setTargetPosition2(tick,
                    new HwDevice[]{drive.driveRightBack, drive.driveLeftFront});
            drive.setTargetPosition2(-tick,
                    new HwDevice[]{drive.driveRightFront, drive.driveLeftBack});
        } else if (dir ==TurnType.TURN_SWERVE_FWD_TURN){
            drive.setTargetPosition2(tick,
                    new HwDevice[]{drive.driveRightBack, drive.driveLeftBack});
            drive.setTargetPosition2(-tick,
                    new HwDevice[]{drive.driveRightFront, drive.driveLeftFront});
        } else {
            drive.setTargetPosition2(-tick);

        }

        long startTimestamp = System.currentTimeMillis();
        long timeStamp = startTimestamp;
        Stalled stalled = new Stalled();
        while (!isStopRequested() && opModeIsActive() &&
                // is RUN_TO_POSITION
                drive.isBusy() &&
                (timeStamp - startTimestamp < timeoutMillis)) {

            int average = ((int)Math.signum(tick))*drive.getEncoder(dir);
            int delta = tick - average;
            if (Math.abs(delta) < 50)
            {
                // done
                break;
            }

            float basePower;
            if (!isAccelerate)
            {
                basePower = Math.signum(delta) * maxPower;
            }
            else
            {
                if (Math.abs(average) < Math.abs(tick / 3))
                {
                    // ramp up for 1/3 of distance
                    // minimum power is START_POWER
                    // base is 1600
                    basePower = (average/1600f) + Math.signum(tick) * POWER_START;
                }
                else
                {
                    // ramp down for 2/3 of distance
                    // minimum power is START_POWER
                    // base is 3200
                    basePower = (delta/3200f) + Math.signum(delta) * POWER_STOP;
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
                drive.setPower(basePower,
                        basePower,
                        dir);

            } // else
            if (lowerLatch) {
                boolean isSwitchDown = robot.switchSlideDown.isTouch();
                if (!isSwitchDown && System.currentTimeMillis()-startTimestamp>250) robot.motorLatch.setPower(-1);
            }
            if (pop) {
                robot.motorIntake.setPower(1);
                robot.motorPopper.setPower(1);

            }
            if (raiseLatch) {
                int positionHang = robot.motorLatch.getCurrentPosition();
                if (positionHang < 6000) {
                    robot.motorLatch.setPower(1);
                } else {
                    robot.motorLatch.setPower(0);
                }
            }
            if (popSlides) {
                popperSlide(0.85f);
            }
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
    /*
        Function used to knock off gold mineral in autonomous
        Total time taken 4 seconds
     */
    private void intakeMineral()
            throws InterruptedException{
        try {
            int positionSlideStart = robot.motorSlide.getCurrentPosition();
            long startTime = System.currentTimeMillis();
            //bringing slide out
            while (robot.motorSlide.getCurrentPosition() - positionSlideStart < (500)
                    && System.currentTimeMillis() - startTime < (1200) && opModeIsActive() && !isStopRequested()) {
                robot.motorSlide.setPower(-1);
                robot.motorIntake.setPower(1);
                popperSlide(0.85f);
                //encoder reset
                idle();
                if (isStopRequested()) {
                    robot.close1();
                    break;
                }
            }
            popperSlide(0);
            boolean isSlideBack = robot.switchSlideIn.isTouch();
            //bringing the slide back in
            startTime = System.currentTimeMillis();
            while (!isSlideBack && System.currentTimeMillis() - startTime < 1800 && opModeIsActive() && !isStopRequested()) {
                robot.motorSlide.setPower(1);
                isSlideBack = robot.switchSlideIn.isTouch();
                robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_OUT);
                robot.motorIntake.setPower(1);
                popperSlide(0.85f);
                //if the hang is down, start raising the popper
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
        }
    }
    /*
    Function used to dump team marker
    Total Time 4.5 seconds
     */
    private void dumpTM(boolean lowerLatch)
        throws InterruptedException{
        try {
            //get the initial positions/times
            int positionSlideStart = robot.motorSlide.getCurrentPosition();
            long startTime = System.currentTimeMillis();
            //take the slide out
            boolean isSlideDown = robot.switchSlideDown.isTouch();
            while (robot.motorSlide.getCurrentPosition() - positionSlideStart < 1000 &&
                    System.currentTimeMillis() - startTime < 1200 && opModeIsActive() && !isStopRequested()) {
                robot.motorSlide.setPower(-1);
                isSlideDown = robot.switchSlideDown.isTouch();
                if (lowerLatch && !isSlideDown) {
                    robot.motorLatch.setPower(-1);
                }
                idle();
                if (isStopRequested()) {
                    robot.close1();
                }
            }
            robot.motorLatch.setPower(-1);
            //dump the team mineral
            try {
                robot.motorIntake.setPower(-1f);
                Thread.sleep(800);
            } catch (InterruptedException e) {robot.close();}
            boolean isSlideBack = robot.switchSlideIn.isTouch();
            startTime = System.currentTimeMillis();
            //bring slide back
            while (!isSlideBack && System.currentTimeMillis() - startTime < 2500 && opModeIsActive()&& !isStopRequested()) {
                robot.motorIntake.setPower(0);
                robot.motorSlide.setPower(1);
                isSlideBack = robot.switchSlideIn.isTouch();
                robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_OUT);
                if (isStopRequested()) {
                    robot.close1();
                }
                idle();
                isSlideDown = robot.switchSlideDown.isTouch();
                if (lowerLatch && opModeIsActive() && !isSlideDown) {
                    robot.motorLatch.setPower(-1);
                } else if (lowerLatch) {
                    popperSlide(0.85f);
                } else {
                    robot.motorLatch.setPower(0);
                    popperSlide(0);
                }
            }
            robot.motorLatch.setPower(0);
            robot.motorSlide.setPower(0);
            popperSlide(0);
        } catch (InterruptedException e) {
            robot.close();
        }
    }
    /*
        Function used to lower latch and raise popper slides, and score gold mineral
        Total Time Taken 6 seconds
     */
    private void scoreMineral()
            throws InterruptedException{
        try {
            popperSlide(0.85f);
            robot.motorPopper.setPower(1);
            Thread.sleep(2400);
            robot.motorPopper.setPower(0);
            robot.motorIntake.setPower(0);
            popperSlide(0);
            robot.servoDump.setPosition(RobotRevAmpedConstants.SERVO_DUMP);
            Thread.sleep(600);
        } catch (InterruptedException e) {
            robot.close1();
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
                SelectLinear sl = new SelectLinear(this);
                boolean isHang = sl.selectHang();
                boolean isSample = sl.selectSample();
                boolean isScore = sl.selectScore();

                telemetry.addData("isHang?", isHang ? "Yes" : "No");
                telemetry.addData("isSample?", isSample ? "Yes" : "No");
                telemetry.addData("isScore?", isScore ? "Yes" : "No");
                telemetry.update();
                robot.drawLed();
                robot.ledGreen.on();
                robot.gyroSensor.resetHeading();
                WaitLinear lp = new WaitLinear(this);
                robot.motorLatch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
                long startTime = System.currentTimeMillis();
                //Landing
                robot.motorLatch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                SampleOrderDetector.GoldenOrder order = SampleOrderDetector.GoldenOrder.UNKNOWN;
                boolean sampled = false;
                if (isHang) {
                    //Sampling
                    long time = startTime;
                    boolean isTouchUp = robot.switchSlideUp.isTouch();
                    while (!isTouchUp && time - startTime < 5250 && opModeIsActive() && !isStopRequested()) {
                        robot.motorLatch.setPower(1);
                        time = System.currentTimeMillis();
                        isTouchUp = robot.switchSlideUp.isTouch();
                        idle();
                        if (isStopRequested()) {
                            robot.close1();
                        }
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
                            if (order == SampleOrderDetector.GoldenOrder.UNKNOWN) {
                                order = SampleOrderDetector.GoldenOrder.RIGHT;
                            }
                            telemetry.addData("Gold", order);
                            long endAnalyzeTime = System.currentTimeMillis();
                            telemetry.addData("Time to Analyze", -startAnalyzeTime + endAnalyzeTime);
                            telemetry.update();
                            sampled = true;
                        }
                    }
                    robot.motorLatch.setPower(0);
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
                    if (order == SampleOrderDetector.GoldenOrder.UNKNOWN) {
                        order = SampleOrderDetector.GoldenOrder.RIGHT;
                    }
                    telemetry.addData("Gold", order);
                    long endAnalyzeTime = System.currentTimeMillis();
                    telemetry.addData("Time to Analyze", -startAnalyzeTime + endAnalyzeTime);
                    telemetry.update();
                    sampled = true;
                }
                popperSlide(0);
                    //drive forward and dump the team marker, make sure the slide clears the center  ball/mineral
                    moveToEncoderDepot(TurnType.FORWARD,
                        17,
                        0.5f,
                        2000,
                        false,
                        true,
                        true,
                        false,
                        false,
                            false);
                    //dump team marker
                    robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_IN);
                    lp.waitMillis(200);
                    dumpTM(true);
                    //check if stop requested
                    if (isStopRequested()) {
                        robot.close1();
                        robot.close();
                    }
                    //drive back to knock off gold mineral
                moveToEncoderDepot(TurnType.FORWARD,
                        -9,
                        0.5f,
                        2000,
                        false,
                        true,
                        false,
                        false,
                        true,
                        false);
                    if (!(order == SampleOrderDetector.GoldenOrder.CENTER)) {
                        //Moving to position to intake the gold mineral
                        if (order == SampleOrderDetector.GoldenOrder.LEFT) {
                            //position robot to intake mineral
                            //THIS AREA MAY NEED CHANGE
                            //THIS AREA MAY NEED CHANGE
                            if (opModeIsActive() && !isStopRequested()) popperSlide(0.85f);
                            drive.turn2(TurnType.TURN_REGULAR,
                                    15,
                                    0.45f,
                                    4000,
                                    false);
                            robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_IN);
                            lp.waitMillis(200);
                            //intake the gold mineral
                            intakeMineral();
                            //move into position to park
                            drive.turn2(TurnType.TURN_REGULAR,
                                    -15,
                                    0.45f,
                                    4000,
                                    false);
                        } else {
                            //position robot to intake mineral, mineral is right
                            if (opModeIsActive() && !isStopRequested()) popperSlide(0.85f);
                            drive.turn2(TurnType.TURN_REGULAR,
                                    -14,
                                    0.5f,
                                    3000,
                                    false);
                            //intake the gold mineral
                            robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_IN);
                            lp.waitMillis(200);
                            intakeMineral();
                            drive.turn2(TurnType.TURN_REGULAR,
                                    15,
                                    0.45f,
                                    4000,
                                    false);
                        }
                    } else {
                        //gold mineral is center, intake gold mineral
                        robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_IN);
                        lp.waitMillis(200);
                        intakeMineral();
                    }
                    //check if stop requested
                    if (isStopRequested()) {
                        robot.close1();
                        robot.close();
                    }
                popperSlide(0);
                if (isScore) {
                    long endTime = System.currentTimeMillis();
                    while(System.currentTimeMillis()-endTime<3000 && opModeIsActive() && !isStopRequested()) {
                        robot.motorPopper.setPower(1);
                        robot.motorIntake.setPower(1);
                        popperSlide(0.85f);
                        int positionHang = robot.motorLatch.getCurrentPosition();
                        if (positionHang < midHangPos && System.currentTimeMillis()-endTime>1000) {
                            robot.motorLatch.setPower(1);
                        } else {
                            robot.motorLatch.setPower(0);
                        }
                    }
                    robot.motorIntake.setPower(0);
                    robot.motorPopper.setPower(0);
                    robot.motorLatch.setPower(0);
                    popperSlide(0);

                    drive.moveToEncoderInch(TurnType.FORWARD,
                            -4,
                            0.5f,
                            1000,
                            false,
                            false,
                            true);
                    drive.moveToEncoderInch(TurnType.STRAFE_LEFT_DIAG,
                            -6,
                            0.5f,
                            1000,
                            false,
                            false,
                            true);
                        //score gold mineral
                        robot.servoDump.setPosition(RobotRevAmpedConstants.SERVO_DUMP);
                        lp.waitMillis(600);
                        robot.servoDump.setPosition(RobotRevAmpedConstants.SERVO_DUMP_UP);
                    drive.moveToEncoderDepot(TurnType.TURN_SWERVE_FWD_TURN,
                            TurnType.FORWARD,
                            -72,
                            42,
                            0.6f,
                            4000,
                            4000,
                            false,
                            false,
                            true);
                } else {
                    long endTime = System.currentTimeMillis();
                    while(System.currentTimeMillis()-endTime<3000 && opModeIsActive() && !isStopRequested()) {
                        popperSlide(0.85f);
                        int positionHang = robot.motorLatch.getCurrentPosition();
                        if (positionHang < midHangPos && System.currentTimeMillis()-endTime>1000) {
                            robot.motorLatch.setPower(1);
                        } else {
                            robot.motorLatch.setPower(0);
                        }
                    }

                    robot.motorIntake.setPower(0);
                    robot.motorPopper.setPower(0);
                    robot.motorLatch.setPower(0);
                    popperSlide(0);

                    drive.moveToEncoderInch(TurnType.FORWARD,
                            -4,
                            0.5f,
                            1000,
                            false,
                            false,
                            true);
                    drive.moveToEncoderInch(TurnType.STRAFE_LEFT_DIAG,
                            -6,
                            0.5f,
                            1000,
                            false,
                            false,
                            true);
                    //score gold mineral
                    robot.servoDump.setPosition(RobotRevAmpedConstants.SERVO_DUMP);
                    lp.waitMillis(600);
                    robot.servoDump.setPosition(RobotRevAmpedConstants.SERVO_DUMP_UP);
                    drive.moveToEncoderDepot(TurnType.TURN_SWERVE_FWD_TURN,
                            TurnType.FORWARD,
                            -72,
                            42,
                            0.6f,
                            4000,
                            4000,
                            false,
                            false,
                            true);
                }
                ///*
                //changed for safe parking on depot side 4/17
                drive.turn2(TurnType.TURN_REGULAR,
                        30,
                        0.4f,
                        3000,
                        true);
                drive.moveToEncoderInch(TurnType.STRAFE,
                        8,
                        0.6f,
                        2000,
                        false,
                        false,
                        true);
                robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_IN);
                drive.moveToEncoderInch(TurnType.FORWARD,
                        10,
                        0.6f,
                        2000,
                        false,
                        false,
                        false);
                //*/
                if (!isScore) {
                    long raiseLatchTime = System.currentTimeMillis();
                    while(System.currentTimeMillis()-raiseLatchTime<3000 && opModeIsActive() && !isStopRequested()) {
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
                robot.motorLatch.setPower(0);
                telemetry.addLine("Six Glyph Auto Done!!!!!");
                telemetry.update();
                RobotLog.i("AutonomousRevamped finish in " + (System.currentTimeMillis() - startTime) + " milliseconds");
                lp.waitMillis(100);
            }
        } catch (InterruptedException e) {
            robot.close1();
        }
    }
}
