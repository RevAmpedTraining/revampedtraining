package org.firstinspires.ftc.teamcode.RevAmpedGame;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.revAmped.components.Button;
import com.revAmped.components.HwLed;
import com.revAmped.components.HwServo;
import com.revAmped.components.HwSwitch;
import com.revAmped.components.SwerveDrive;
import com.revAmped.config.RobotRevAmpedConstants;
import com.revAmped.config.SwerveDriveConstants;
import com.revAmped.linear.components.RobotRevAmpedLinear;
import com.revAmped.linear.components.RobotRevAmpedLinear2;
import com.revAmped.linear.components.SwerveDriveLinear;
import com.revAmped.linear.components.RobotRevAmpedLinearTest;
import com.revAmped.test.ServoTestInfo;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.text.DecimalFormat;
import java.util.List;

/**
 * Created by John Wang on 10/14/2018.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Tester", group="Game")
public class TesterRevAmpedSwerve
        extends LinearOpMode {
    private final static float MIN_SERVO_TICK = 1;
    private final static DecimalFormat numberFormatter = new DecimalFormat("######");

    private RobotRevAmpedLinearTest robot;
    private SwerveDriveLinear drive;

    private ServoTestInfo[] servoTestInfos;

    private int servoTestCounter = 0;

    private int servoCalibrateCounter = 0;

    //vuforia test variables
    // public static final String TAG = "Vuforia VuMark Sample";
    //OpenGLMatrix lastLocation = null;
    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    public enum ETest {
        NONE,
        SWITCH,
        ENCODER,
        SERVO_CALIBRATE,
        SERVO,
        DRIVE,
        LED,
        POP_SLIDE,
        SLIDE,
        GYRO,
        LATCH;

        private static int numberTests = 0;

        public static TesterRevAmpedSwerve.ETest getTest(int ordinal) {
            for (TesterRevAmpedSwerve.ETest e : values()) {
                if (e.ordinal() == ordinal) {
                    return e;
                }
            }

            return NONE;
        }

        public static int getNumberTests() {
            if (numberTests == 0) {
                for (TesterRevAmpedSwerve.ETest e : values()) {
                    numberTests++;
                }
            }
            return numberTests;
        }
    }

    @Override
    public void runOpMode()
            throws InterruptedException {
        robot = new RobotRevAmpedLinearTest(this);
        drive = robot.getSwerveDriveLinear();

        servoTestInfos = new ServoTestInfo[]{
                new ServoTestInfo(robot.servoRightFront,
                        SwerveDriveConstants.SERVO_RIGHTFRONT_START,
                        SwerveDriveConstants.SERVO_RIGHTFRONT_END),
                new ServoTestInfo(robot.servoRightBack,
                        SwerveDriveConstants.SERVO_RIGHTBACK_START,
                        SwerveDriveConstants.SERVO_RIGHTBACK_END),
                new ServoTestInfo(robot.servoLeftFront,
                        SwerveDriveConstants.SERVO_LEFTFRONT_START,
                        SwerveDriveConstants.SERVO_LEFTFRONT_END),
                new ServoTestInfo(robot.servoLeftBack,
                        SwerveDriveConstants.SERVO_LEFTBACK_START,
                        SwerveDriveConstants.SERVO_LEFTBACK_END),
                new ServoTestInfo(robot.servoLatch,
                        RobotRevAmpedConstants.SERVO_LATCH_IN,
                        RobotRevAmpedConstants.SERVO_LATCH_OUT),
                new ServoTestInfo(robot.servoDump,
                        RobotRevAmpedConstants.SERVO_DUMP_INIT,
                        RobotRevAmpedConstants.SERVO_DUMP)

        };

        int testCounter = 0;
        TesterRevAmpedSwerve.ETest currentTest = TesterRevAmpedSwerve.ETest.NONE;

        telemetry.addData("Waiting", "LinearTester");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            //choosing
            if (gamepad1.right_trigger > 0.9 && Button.BTN_NEXT.canPress(timeStamp)) {
                testCounter++;
                if (testCounter >= TesterRevAmpedSwerve.ETest.getNumberTests()) {
                    testCounter = 0;
                }
                currentTest = TesterRevAmpedSwerve.ETest.getTest(testCounter);
            } else if (gamepad1.left_trigger > 0.9 && Button.BTN_PREV.canPress(timeStamp)) {
                testCounter--;
                if (testCounter < 0) {
                    testCounter = TesterRevAmpedSwerve.ETest.getNumberTests() - 1;
                }
                currentTest = TesterRevAmpedSwerve.ETest.getTest(testCounter);
            }

            telemetry.addData("Test", currentTest);
            telemetry.addData("Select", "Next:RightTrigger Prev:LeftTrigger");
            telemetry.addData("Confirm", "Start");

            // test loop
            if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {
                switch (currentTest) {
                    case SWITCH:
                        switchTest();
                        break;
                    case ENCODER:
                        encoderTest();
                        break;
                    case SERVO_CALIBRATE:
                        servoCalibrate(robot.servos);
                        break;
                    case DRIVE:
                        driveTest();
                        break;
                    case SERVO:
                        servoTest(servoTestInfos);
                        break;
                    case LED:
                        ledTest();
                        break;
                    case GYRO:
                        gyroTest();
                        break;
                    case SLIDE:
                        intakeSlideTest();
                    case POP_SLIDE:
                        popSlideTest();
                    case LATCH:
                        latchSlideTest();
                    case NONE:
                    default:
                        break;
                }
            }
            telemetry.update();
            idle();
        }
    }

   private void switchTest()
            throws InterruptedException {
        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }

            for (HwSwitch s : robot.switchs) {
                telemetry.addData(s.toString(), Boolean.toString(s.isTouch()));
            }

            telemetry.addData("Stop", "Back");

            telemetry.update();
            idle();
        }
    }
    private void encoderTest()
            throws InterruptedException {
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.motorSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.motorLatch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.motorPopper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {

                drive.resetPosition();
                robot.motorSlide.resetPosition();
                robot.motorPopper.resetPosition();
                robot.motorLatch.resetPosition();
                robot.motorIntake.resetPosition();
                idle();
            } else if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }
            if (gamepad2.a) {
                robot.motorPopper.setPower(0.5f);
                Thread.sleep(500);
            } else {
                robot.motorPopper.setPower(0);
            }

            telemetry.addData("Front",
                    "Left:" + numberFormatter.format(robot.driveLeftFront.getCurrentPosition()) +
                            " Right:" + numberFormatter.format(robot.driveRightFront.getCurrentPosition()));
            telemetry.addData("Back",
                    "Left:" + numberFormatter.format(robot.driveLeftBack.getCurrentPosition()) +
                            " Right:" + numberFormatter.format(robot.driveRightBack.getCurrentPosition()));
            telemetry.addData("Intake", numberFormatter.format(robot.motorIntake.getCurrentPosition()));
            telemetry.addData("Popper", numberFormatter.format(robot.motorIntake.getCurrentPosition()));
            telemetry.addData("Latch", numberFormatter.format(robot.motorLatch.getCurrentPosition()));
            telemetry.addData("Slide", numberFormatter.format(robot.motorSlide.getCurrentPosition()));
            telemetry.addData("Reset", "Start");
            telemetry.addData("Stop", "Back");
            telemetry.update();
            idle();
        }

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        idle();
    }

    private void servoCalibrate(List<HwServo> servoCalibrateList)
            throws InterruptedException {
        int posJoy1 = (int) (servoCalibrateList.get(servoCalibrateCounter).getPosition() * 255f);

        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            if (gamepad1.right_trigger > 0.9 && Button.BTN_NEXT.canPress(timeStamp)) {
                servoCalibrateCounter++;
                if (servoCalibrateCounter >= servoCalibrateList.size()) {
                    servoCalibrateCounter = 0;
                }
                posJoy1 = (int) (servoCalibrateList.get(servoCalibrateCounter).getPosition() * 255f);
            } else if (gamepad1.left_trigger > 0.9 && Button.BTN_PREV.canPress(timeStamp)) {
                servoCalibrateCounter--;
                if (servoCalibrateCounter < 0) {
                    servoCalibrateCounter = servoCalibrateList.size() - 1;
                }
                posJoy1 = (int) (servoCalibrateList.get(servoCalibrateCounter).getPosition() * 255f);
            } else if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                return;
            }

            //calibration
            if (gamepad1.x && Button.BTN_MINUS.canPress4Short(timeStamp)) {
                posJoy1 -= MIN_SERVO_TICK;
            } else if (gamepad1.b && Button.BTN_PLUS.canPress4Short(timeStamp)) {
                posJoy1 += MIN_SERVO_TICK;
            } else if (gamepad1.y && Button.BTN_MAX.canPress(timeStamp)) {
                posJoy1 = 255;
            } else if (gamepad1.a && Button.BTN_MIN.canPress(timeStamp)) {
                posJoy1 = 0;
            } else if (gamepad1.right_stick_button && Button.BTN_MID.canPress(timeStamp)) {
                posJoy1 = 128;
            }
            posJoy1 = Range.clip(posJoy1, 0, 255);
            servoCalibrateList.get(servoCalibrateCounter).setPosition(posJoy1 / 255f);

            telemetry.addData("Adjust", "+: B -: X Max: Y Min: A Mid: RStick");
            telemetry.addData("Position", Integer.toString(posJoy1));
            telemetry.addData("Servo", servoCalibrateList.get(servoCalibrateCounter));
            telemetry.addData("Select", "Next: RightTrigger Prev: LeftTrigger");
            telemetry.addData("Stop", "Back");

            telemetry.update();
            idle();
        }
    }

    private void driveTest()
            throws InterruptedException {
        float[][] p = new float[][]{
                {0.25f, 0.25f},
                {-0.25f, -0.25f},
                {0.25f, -0.25f},
                {-0.25f, 0.25f},
        };

        back:
        for (int i = 0; i < 4; i++) {
            drive.resetPosition();

            long startTimestamp = System.currentTimeMillis();
            long timeStamp = startTimestamp;

            while (opModeIsActive() &&
                    timeStamp - startTimestamp < 5000) {

                robot.driveLeftFront.setPower(p[i][0]);
                robot.driveLeftBack.setPower(p[i][0]);
                robot.driveRightFront.setPower((p[i][1]));
                robot.driveRightBack.setPower((p[i][1]));

                telemetry.addData("Front",
                        "Left:" + numberFormatter.format(robot.driveLeftFront.getCurrentPosition()) +
                                " Right:" + numberFormatter.format(robot.driveRightFront.getCurrentPosition()));
                telemetry.addData("Back",
                        "Left:" + numberFormatter.format(robot.driveLeftBack.getCurrentPosition()) +
                                " Right:" + numberFormatter.format(robot.driveRightBack.getCurrentPosition()));
                telemetry.addData("Stop", "Back");

                if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                    break back;
                }

                telemetry.update();
                idle();
                timeStamp = System.currentTimeMillis();
            }

            drive.stop();
        }

        drive.stop();
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        idle();
    }
    private void servoTest(ServoTestInfo[] servoTestInfos)
            throws InterruptedException
    {
        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            if (gamepad1.right_trigger > 0.9 && Button.BTN_NEXT.canPress(timeStamp)) {
                servoTestCounter++;
                if (servoTestCounter >= servoTestInfos.length) {
                    servoTestCounter = 0;
                }
            }
            else if (gamepad1.left_trigger > 0.9 && Button.BTN_PREV.canPress(timeStamp)) {
                servoTestCounter--;
                if (servoTestCounter < 0) {
                    servoTestCounter = servoTestInfos.length - 1;
                }
            }
            else if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                return;
            }
            else if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {
                servoTest(servoTestInfos[servoTestCounter]);
            }

            telemetry.addData("Select", "Next:RightTrigger Prev:LeftTrigger");
            telemetry.addData("Servo", servoTestInfos[servoTestCounter].servo);
            telemetry.addData("Confirm", "Start");
            telemetry.addData("Stop", "Back");

            telemetry.update();

            idle();
        }
    }

    private void intakeSlideTest()
    throws InterruptedException {
        long timeStamp1 = System.currentTimeMillis();
        final float slidePower = 0.75f* RobotRevAmpedConstants.POWER_SLIDE;
        long timestamp = System.currentTimeMillis();
        for (int i = 0; i < 2; i++) {
            while (opModeIsActive() && timestamp-timeStamp1<1250) {
                robot.motorSlide.setPower(slidePower);
                if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp1)) {
                    break;
                }

                telemetry.addData("Test", "Slide");
                telemetry.addData("Stop", "Back");
                telemetry.addData("Slide",
                        numberFormatter.format(robot.motorSlide.getCurrentPosition()));
                telemetry.update();
                idle();
                timestamp = System.currentTimeMillis();
            }

            robot.motorSlide.setPower(0);
            while (opModeIsActive() && !robot.switchSlideIn.isTouch()) {
                robot.motorSlide.setPower(-slidePower);
                if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp1)) {
                    break;
                }

                telemetry.addData("Test", "Slide");
                telemetry.addData("Stop", "Back");
                telemetry.addData("Switch Down", robot.switchSlideIn.isTouch());
                telemetry.addData("Slide",
                        numberFormatter.format(robot.motorSlide.getCurrentPosition()));
                telemetry.update();
                idle();
                timeStamp1 = System.currentTimeMillis();
            }

            robot.motorSlide.setPower(0);
            idle();
        }
    }
    private void popSlideTest()
            throws InterruptedException {
        long timeStamp1 = System.currentTimeMillis();
        final float slidePower = 0.75f* RobotRevAmpedConstants.POWER_SLIDE;
        long timestamp = System.currentTimeMillis();
        for (int i = 0; i < 2; i++) {
            while (opModeIsActive() && timestamp-timeStamp1<1250) {
                robot.servoTelescopeL.setPower(-0.8f);
                robot.servoTelescopeR.setPower(0.8f);
                if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp1)) {
                    break;
                }

                telemetry.addData("Test", "Slide");
                telemetry.addData("Stop", "Back");
                telemetry.addData("Slide",
                        numberFormatter.format(robot.motorSlide.getCurrentPosition()));
                telemetry.update();
                idle();
                timestamp = System.currentTimeMillis();
            }
            timestamp = System.currentTimeMillis();
            robot.servoTelescopeL.setPower(0);
            robot.servoTelescopeR.setPower(0);
            while (opModeIsActive() && System.currentTimeMillis()-timestamp<1250) {
                robot.servoTelescopeL.setPower(0.8f);
                robot.servoTelescopeR.setPower(-0.8f);
                if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp1)) {
                    break;
                }

                telemetry.addData("Test", "Slide");
                telemetry.addData("Stop", "Back");
                telemetry.addData("Switch Down", robot.switchSlideIn.isTouch());
                telemetry.addData("Slide",
                        numberFormatter.format(robot.motorSlide.getCurrentPosition()));
                telemetry.update();
                idle();
                timeStamp1 = System.currentTimeMillis();
            }

            robot.servoTelescopeL.setPower(0);
            robot.servoTelescopeR.setPower(0);
            idle();
        }
    }
    private void latchSlideTest()
            throws InterruptedException {
        long timeStamp1 = System.currentTimeMillis();
        final float slidePower = 0.75f* RobotRevAmpedConstants.POWER_SLIDE;
        boolean isSlideUp = robot.switchSlideUp.isTouch();
        for (int i = 0; i < 2; i++) {
            while (opModeIsActive() && !isSlideUp) {
                robot.motorLatch.setPower(slidePower);
                if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp1)) {
                    break;
                }

                telemetry.addData("Test", "Slide");
                telemetry.addData("Stop", "Back");
                telemetry.addData("Slide",
                        numberFormatter.format(robot.motorSlide.getCurrentPosition()));
                telemetry.update();
                idle();
                isSlideUp = robot.switchSlideUp.isTouch();
            }
            boolean isSlideDown = robot.switchSlideDown.isTouch();
            robot.motorLatch.setPower(0);
            while (opModeIsActive() && !isSlideDown) {
                robot.motorLatch.setPower(-slidePower);
                if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp1)) {
                    break;
                }
                isSlideDown = robot.switchSlideDown.isTouch();
                telemetry.addData("Test", "Slide");
                telemetry.addData("Stop", "Back");
                telemetry.addData("Switch Down", robot.switchSlideIn.isTouch());
                telemetry.addData("Slide",
                        numberFormatter.format(robot.motorSlide.getCurrentPosition()));
                telemetry.update();
                idle();
            }

            robot.motorLatch.setPower(0);
            idle();
        }
    }

    private void servoTest(ServoTestInfo servoTestInfo)
            throws InterruptedException
    {
        for (int i = 0; i <= servoTestInfo.positions.length; i++) {
            float currentPosition = servoTestInfo.servo.getPosition();
            float position;
            if (i < servoTestInfo.positions.length) {
                position = servoTestInfo.positions[i];
            }
            else {
                position = servoTestInfo.servo.getInitialPosition();
            }

            long startTimestamp = System.currentTimeMillis();
            long timeStamp = startTimestamp;
            long timeout = (long)(Math.abs(position - currentPosition) * 1000 * servoTestInfo.timeScale);
            //test
            telemetry.addData("Position", Float.toString(servoTestInfo.servo.getPosition()));
            telemetry.addData("Servo", servoTestInfo.servo);
            //============
            while (opModeIsActive() &&
                    timeStamp - startTimestamp < timeout) {
                servoTestInfo.servo.setPosition(position);

                if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                    return;
                }

                telemetry.addData("Position", Float.toString(servoTestInfo.servo.getPosition()));
                telemetry.addData("Servo", servoTestInfo.servo);
                telemetry.addData("Stop", "Back");
                telemetry.update();
                idle();
                timeStamp = System.currentTimeMillis();
            }
        }
    }
    private void ledTest()
            throws InterruptedException {
        final HwLed.ELedStatus[] ledStatuses = new HwLed.ELedStatus[]{
                HwLed.ELedStatus.LED_ON,
                HwLed.ELedStatus.LED_BLINK,
                HwLed.ELedStatus.LED_OFF};
        for (HwLed led : robot.leds) {

            for (int i = 0; i < ledStatuses.length; i++) {
                long startTimestamp = System.currentTimeMillis();
                long timeStamp = startTimestamp;

                while (opModeIsActive() &&
                        timeStamp - startTimestamp < 2000) {

                    if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                        return;
                    }

                    led.set(ledStatuses[i]);
                    led.draw();
                    telemetry.addData("LED", led + " " + led.toString() + " " + ledStatuses[i]);
                    telemetry.addData("Stop", "Back");

                    telemetry.update();
                    idle();
                    timeStamp = System.currentTimeMillis();
                }
            }
        }
    }

    private void gyroTest()
            throws InterruptedException {
        try {
            while (opModeIsActive()) {
                long timeStamp = System.currentTimeMillis();

                if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {

                    robot.gyroSensor.resetHeading();
                    idle();
                } else if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                    break;
                }

                telemetry.addData("Heading",
                        Float.toString(robot.gyroSensor.getHeading()));
                telemetry.addData("Reset", "Start");
                telemetry.addData("Stop", "Back");

                telemetry.update();
                idle();
            }
            idle();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    }


