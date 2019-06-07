package org.firstinspires.ftc.teamcode.RevAmpedTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import com.revAmped.components.Button;
import com.revAmped.components.Drive;
import com.revAmped.components.HwDevice;
import com.revAmped.components.TurnType;
import com.revAmped.config.RobotRevAmpedConstants;
import com.revAmped.config.SwerveDriveConstants;
import com.revAmped.linear.components.MoveAction;
import com.revAmped.linear.components.RobotRevAmpedLinearTest;
import com.revAmped.linear.components.SwerveDriveLinear;
import com.revAmped.linear.util.SelectLinear;
import com.revAmped.linear.util.WaitLinear;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.revAmped.util.Stalled;

/**
 * Move tester
 */
//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="MoveTesterReVamped", group="Test")
public class SubpartTester
        extends LinearOpMode {
    private RobotRevAmpedLinearTest robot = null;

    private SwerveDriveLinear drive;
    private static final Button BTN_ROLLER_OUT = new Button();

    float POWER_START = 0.17f;
    float POWER_STOP = 0.10f;

    private final int doubleSampleDriveC = 45;

    private final int doubleSampleStrafeC = 18;

    private final int doubleSampleStrafeL = 26;
    //how far away from depot in crater position
    private final int distanceToDepot = 56;

    public enum ETest {
        NONE,
        Curve,
        Depot_Move,
        DOUBLE_SAMPLE_RIGHT,
        DOUBLE_SAMPLE_LEFT,
        DOUBLE_SAMPLE_CENTER;

        private static int numberTests = 0;

        public static SubpartTester.ETest getTest(int ordinal) {
            for (SubpartTester.ETest e : values()) {
                if (e.ordinal() == ordinal) {
                    return e;
                }
            }

            return NONE;
        }

        public static int getNumberTests() {
            if (numberTests == 0) {
                for (SubpartTester.ETest e : values()) {
                    numberTests++;
                }
            }
            return numberTests;
        }
    }

    @Override
    public void runOpMode()
            throws InterruptedException {
        try {
            // init
            robot = new RobotRevAmpedLinearTest(this);
            drive = robot.getSwerveDriveLinear();
            run();
        } finally {
            // shut down
            if (robot != null) {
                robot.close();
            }
        }
    }
    private void popperSlide(float p)
            throws InterruptedException {
        robot.servoTelescopeR.setPower(p);
        robot.servoTelescopeL.setPower(-p);
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
            robot.servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_CRABLEFT+25/255f);
            robot.servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_CRABLEFT+25/255f);

            robot.servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_CRABLEFT+10/255f);
            robot.servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_CRABLEFT+10/255f);
        } else {
            robot.servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_CRABRIGHT/*-8/255f*/);
            robot.servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_CRABRIGHT/*-8/255f*/);

            robot.servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_CRABRIGHT-12/255f);
            robot.servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_CRABRIGHT-12/255f);
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
                if (Math.abs(delta) < Math.abs((startDelta / 1.25))) {
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
                    drive.setPowerCurve(basePower+0.12f, basePower);
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
    /**
     * autonomous function
     *
     * @throws InterruptedException
     */
    public void run()
            throws InterruptedException {

        long timestamp = System.currentTimeMillis();
        int testCounter = 0;
        SubpartTester.ETest currentTest = SubpartTester.ETest.NONE;

        telemetry.addData("Waiting", "LinearTester");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            //choosing
            if (gamepad1.right_trigger > 0.9 && Button.BTN_NEXT.canPress(timeStamp)) {
                testCounter++;
                if (testCounter >= SubpartTester.ETest.getNumberTests()) {
                    testCounter = 0;
                }
                currentTest = SubpartTester.ETest.getTest(testCounter);
            } else if (gamepad1.left_trigger > 0.9 && Button.BTN_PREV.canPress(timeStamp)) {
                testCounter--;
                if (testCounter < 0) {
                    testCounter = SubpartTester.ETest.getNumberTests() - 1;
                }
                currentTest = SubpartTester.ETest.getTest(testCounter);
            }

            telemetry.addData("Test", currentTest);
            telemetry.addData("Select", "Next:RightTrigger Prev:LeftTrigger");
            telemetry.addData("Confirm", "Start");

            // test loop
            if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {
                switch (currentTest) {
                    case Curve:
                        curveTest();
                        break;
                    case Depot_Move:
                        depotTest();
                        break;
                    case DOUBLE_SAMPLE_CENTER:
                        dsCenter();
                    case DOUBLE_SAMPLE_LEFT:
                        dsLeft();
                    case DOUBLE_SAMPLE_RIGHT:
                        dsRight();
                    case NONE:
                    default:
                        break;
                }
            }
            telemetry.update();
            idle();
        }
    }

    private WaitLinear lp = new WaitLinear(this);

    private void curveTest()
        throws InterruptedException {
        lp.waitMillis(1000);
        moveToEncoderCrater(TurnType.CURVE,
                -45,
                0.65f,
                4000,
                false,
                true,
                false,
                false,
                true,
                true,
                false);
        telemetry.addLine("Done");
        telemetry.update();
        lp.waitMillis(1000);
    }

    private void depotTest()
        throws InterruptedException {
        lp.waitMillis(1000);
        drive.moveToEncoderDepot(TurnType.TURN_SWERVE_FWD_TURN,
                TurnType.FORWARD,
                -67,
                85,
                0.6f,
                6000,
                6000,
                false,
                false,
                true);
        telemetry.addLine("Done");
        telemetry.update();
        lp.waitMillis(1000);
    }

    private void dsRight ()
        throws InterruptedException {
        moveToEncoderCrater(TurnType.CURVE,
                -45,
                0.65f,
                4000,
                false,
                true,
                false,
                false,
                true,
                true,
                false);
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
        moveToEncoderCrater(TurnType.FORWARD_RIGHT,
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

    private void dsLeft()
        throws InterruptedException {
        moveToEncoderCrater(TurnType.CURVE,
                -45,
                0.65f,
                4000,
                false,
                true,
                false,
                false,
                true,
                true,
                false);
        moveToEncoderCrater(TurnType.FORWARD_TM,
                -distanceToDepot - 3,
                0.7f,
                4000,
                false,
                true,
                false,
                false,
                false,
                false,
                false);
        moveToEncoderCrater(TurnType.STRAFE,
                doubleSampleStrafeL,
                0.7f,
                3000,
                false,
                false,
                false,
                false,
                false,
                false,
                false);
        moveToEncoderCrater(TurnType.STRAFE,
                -doubleSampleStrafeL - 3,
                0.6f,
                3000,
                false,
                true,
                false,
                false,
                false,
                false,
                false);
        //robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_IN);
        //team marker dump
        robot.servoTeam.setPosition(RobotRevAmpedConstants.SERVO_MARKER_DUMP);
        moveToEncoderCrater(TurnType.FORWARD,
                50,
                0.9f,
                5000,
                true,
                true,
                false,
                false,
                false,
                false,
                false);
    }

    private void dsCenter ()
        throws InterruptedException {
        moveToEncoderCrater(TurnType.CURVE,
                -45,
                0.65f,
                4000,
                false,
                true,
                false,
                false,
                true,
                true,
                false);
        moveToEncoderCrater(TurnType.FORWARD_TM,
                -doubleSampleDriveC,
                0.7f,
                4000,
                false,
                true,
                false,
                false,
                false,
                false,
                false);
        moveToEncoderCrater(TurnType.STRAFE,
                doubleSampleStrafeC ,
                0.7f,
                3000,
                false,
                false,
                false,
                false,
                false,
                false,
                false);
        moveToEncoderCrater(TurnType.STRAFE,
                -doubleSampleStrafeC - 3,
                0.6f,
                3000,
                false,
                true,
                false,
                false,
                false,
                false,
                false);
        //robot.servoLatch.setPosition(RobotRevAmpedConstants.SERVO_LATCH_IN);
        //team marker dump
        robot.servoTeam.setPosition(RobotRevAmpedConstants.SERVO_MARKER_DUMP);
        moveToEncoderCrater(TurnType.FORWARD,
                50,
                0.9f,
                5000,
                true,
                true,
                false,
                false,
                false,
                false,
                false);
    }

}



