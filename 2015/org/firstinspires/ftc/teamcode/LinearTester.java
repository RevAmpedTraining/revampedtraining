package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.revAmped.components.HwColorSensor;
import com.revAmped.config.RobotConstants;
import com.revAmped.components.Button;
import com.revAmped.components.HwLed;
import com.revAmped.components.HwServo;
import com.revAmped.linear.components.RobotLinear;
import com.revAmped.linear.components.TankDriveLinear;
import com.revAmped.linear.util.SelectLinear;
import com.revAmped.util.Delayed;
import com.revAmped.linear.util.WaitLinear;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.text.DecimalFormat;
import java.util.List;

/**
 * Robot Tester
 */
@TeleOp(name="Tester", group="Test")
public class LinearTester
    extends LinearOpMode {

    private final static float MIN_SERVO_TICK = 0.005f;
    private final static DecimalFormat numberFormatter = new DecimalFormat("######");

    private RobotLinear robot;
    private TankDriveLinear drive;

    private ServoTestInfo[] servoTestInfos;
    private ServoTestInfo[] bucketServoTestInfos;
    private HwColorSensor[] colorSensors;

    private int servoTestCounter = 0;

    private int servoCalibrateCounter = 0;

    public enum ETest {
        NONE,
        SWITCH,
        ENCODER,
        SERVO_CALIBRATE,
        SERVO,
        DRIVE,
        SLIDE,
        ROLLER,
        LED,
        GYRO,
        COLOR,
        BEACON;

        private static int numberTests = 0;

        public static ETest getTest(int ordinal)
        {
            for (ETest e : values()) {
                if (e.ordinal() == ordinal) {
                    return e;
                }
            }

            return NONE;
        }

        public static int getNumberTests() {
            if (numberTests == 0) {
                for (ETest e : values()) {
                    numberTests++;
                }
            }
            return numberTests;
        }
    }

    @Override
    public void runOpMode()
        throws InterruptedException
    {
        // init
        robot = new RobotLinear(this);
        drive = robot.getTankDriveLinear();
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.roller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoTestInfos = new ServoTestInfo[] {
            new ServoTestInfo(robot.servoFrontDoor,
                              RobotConstants.SERVO_FDOOR_EASE,
                              RobotConstants.SERVO_FDOOR_CLOSED,
                              RobotConstants.SERVO_FDOOR_PUSH),
            new ServoTestInfo(robot.servoBackDoor,
                              RobotConstants.SERVO_BDOOR_PUSH),
            new ServoTestInfo(robot.servoBucket,
                              RobotConstants.SERVO_BUCKET_OPEN),
            new ServoTestInfo(robot.servoHookLeft,
                              RobotConstants.SERVO_HOOKL_DOWN),
            new ServoTestInfo(robot.servoHookRight,
                              RobotConstants.SERVO_HOOKR_DOWN),
            new ServoTestInfo(robot.servoSlide,
                              RobotConstants.SERVO_SLIDE_END),
            new ServoTestInfo(robot.servoClimber,
                              RobotConstants.SERVO_CLIMBER_OUT),
            new ServoTestInfo(robot.servoWingLeft,
                              RobotConstants.SERVO_WINGL_OUT),
            new ServoTestInfo(robot.servoWingRight,
                              RobotConstants.SERVO_WINGR_OUT),
            new ServoTestInfo(robot.servoBeacon,
                              RobotConstants.SERVO_BEACON_LEFT,
                              RobotConstants.SERVO_BEACON_RIGHT),
        };

        bucketServoTestInfos = new ServoTestInfo[] {
            new ServoTestInfo(robot.servoBucketTurn,
                              8,
                              RobotConstants.SERVO_BUCKET_TURN_LEFT,
                              RobotConstants.SERVO_BUCKET_TURN_RIGHT),
            new ServoTestInfo(robot.servoBucket,
                              RobotConstants.SERVO_BUCKET_OPEN,
                              RobotConstants.SERVO_BUCKET_OPEN_WIDE),
        };

        colorSensors = new HwColorSensor[] {
            robot.colorLeftOut,
            robot.colorLeftIn,
            robot.colorRightIn,
            robot.colorRightOut,
        };

        int testCounter = 0;
        ETest currentTest = ETest.NONE;

        telemetry.addData("Waiting", "LinearTester");
        waitForStart();

        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            //choosing
            if(gamepad1.right_trigger > 0.9 && Button.BTN_NEXT.canPress(timeStamp)) {
                testCounter++;
                if(testCounter >= ETest.getNumberTests()){
                    testCounter = 0;
                }
                currentTest = ETest.getTest(testCounter);
            } else if(gamepad1.left_trigger > 0.9 && Button.BTN_PREV.canPress(timeStamp)) {
                testCounter--;
                if(testCounter < 0){
                    testCounter = ETest.getNumberTests() - 1;
                }
                currentTest = ETest.getTest(testCounter);
            }

            telemetry.addData("Test", currentTest);
            telemetry.addData("Select", "Next:RightTrigger Prev:LeftTrigger");
            telemetry.addData("Confirm", "Start");

            // test loop
            if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {
                switch(currentTest) {
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
                    case SLIDE:
                        slideTest();
                        break;
                    case ROLLER:
                        rollerTest();
                        break;
                    case SERVO:
                        servoTest(servoTestInfos);
                        break;
                    case LED:
                        ledTest(robot.leds);
                        break;
                    case GYRO:
                        gyroTest();
                        break;
                    case COLOR:
                        colorTest();
                        break;
                    case BEACON:
                        beaconTest();
                        break;
                    case NONE:
                    default:
                        break;
                }
            }

            idle();
        }
    }

    private void switchTest ()
        throws InterruptedException
    {
        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            if (gamepad1.back && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }

            telemetry.addData("Back", Boolean.toString(robot.switchSlideDown.isTouch()));
            telemetry.addData("Bucket", Boolean.toString(robot.switchBucket.isTouch()));
            telemetry.addData("Front", Boolean.toString(robot.switchSlideUp.isTouch()));
            telemetry.addData("Stop", "Back");

            idle();
        }
    }

    private void encoderTest ()
        throws InterruptedException
    {
        Delayed delay = new Delayed();
        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {

                drive.resetPosition();
                robot.slideLeft.resetPosition();
                robot.slideRight.resetPosition();
                idle();
            }
            else if (gamepad1.back && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }

            if (robot.switchSlideDown.isTouch()) {
                if (delay.isDelayed(timeStamp)) {
                    delay.reset();
                    robot.slideLeft.resetPosition();
                    robot.slideRight.resetPosition();
                }
            }

            telemetry.addData("Front",
                              "Left:" + numberFormatter.format(robot.driveLeftFront.getCurrentPosition()) +
                                  " Right:" + numberFormatter.format(robot.driveRightFront.getCurrentPosition()));
            telemetry.addData("Back",
                              "Left:" + numberFormatter.format(robot.driveLeftBack.getCurrentPosition()) +
                                  " Right:" + numberFormatter.format(robot.driveRightBack.getCurrentPosition()));
            telemetry.addData("Slide",
                              "Left:" + numberFormatter.format(robot.slideLeft.getCurrentPosition()) +
                                  " Right:" + numberFormatter.format(robot.slideRight.getCurrentPosition()));
            telemetry.addData("Reset", "Start");
            telemetry.addData("Stop", "Back");

            idle();
        }

        idle();
    }

    private void servoCalibrate(List<HwServo> servoCalibrateList)
        throws InterruptedException
    {
        float posJoy1 = servoCalibrateList.get(servoCalibrateCounter).getPosition();

        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            if(gamepad1.right_trigger > 0.9 && Button.BTN_NEXT.canPress(timeStamp)) {
                servoCalibrateCounter++;
                if(servoCalibrateCounter >= servoCalibrateList.size()){
                    servoCalibrateCounter = 0;
                }
                posJoy1 = servoCalibrateList.get(servoCalibrateCounter).getPosition();
            } else if(gamepad1.left_trigger > 0.9 && Button.BTN_PREV.canPress(timeStamp)) {
                servoCalibrateCounter--;
                if(servoCalibrateCounter < 0){
                    servoCalibrateCounter = servoCalibrateList.size() - 1;
                }
                posJoy1 = servoCalibrateList.get(servoCalibrateCounter).getPosition();
            }
            else if (gamepad1.back && Button.BTN_BACK.canPress(timeStamp)) {
                return;
            }

            //calibration
            if (gamepad1.x && Button.BTN_MINUS.canPress4Short(timeStamp)) {
                posJoy1 -= MIN_SERVO_TICK;
            } else if (gamepad1.b && Button.BTN_PLUS.canPress4Short(timeStamp)) {
                posJoy1 += MIN_SERVO_TICK;
            }
            posJoy1 = Range.clip(posJoy1, 0, 1);
            servoCalibrateList.get(servoCalibrateCounter).setPosition(posJoy1);

            telemetry.addData("Adjust", "+:B -:X");
            telemetry.addData("Position", Float.toString(posJoy1));
            telemetry.addData("Servo", servoCalibrateList.get(servoCalibrateCounter));
            telemetry.addData("Select", "Next:RightTrigger Prev:LeftTrigger");
            telemetry.addData("Stop", "Back");

            idle();
        }
    }

    private void driveTest ()
        throws InterruptedException
    {
        float[][] p = new float[][]{
            {0.25f, 0.25f, 0.25f, 0.25f},
            {-0.25f, -0.25f, -0.25f, -0.25f},
            {0.25f, 0.25f, -0.25f, -0.25f},
            {-0.25f, -0.25f, 0.25f, 0.25f},
        };

        back:
        for (int i = 0; i < 4; i++) {
            drive.resetPosition();

            long startTimestamp = System.currentTimeMillis();
            long timeStamp = startTimestamp;

            while (opModeIsActive() &&
                timeStamp - startTimestamp < 5000) {

                robot.driveLeftFront.setPower(p[i][0]);
                robot.driveRightFront.setPower(p[i][1]);
                robot.driveLeftBack.setPower(p[i][2]);
                robot.driveRightBack.setPower(p[i][3]);

                telemetry.addData("Front",
                                  "Left:" + numberFormatter.format(robot.driveLeftFront.getCurrentPosition()) +
                                      " Right:" + numberFormatter.format(robot.driveRightFront.getCurrentPosition()));
                telemetry.addData("Back",
                                  "Left:" + numberFormatter.format(robot.driveLeftBack.getCurrentPosition()) +
                                      " Right:" + numberFormatter.format(robot.driveRightBack.getCurrentPosition()));
                telemetry.addData("Stop", "Back");

                if (gamepad1.back && Button.BTN_BACK.canPress(timeStamp)) {
                    break back;
                }

                idle();
                timeStamp = System.currentTimeMillis();
            }

            drive.stop();
        }

        drive.stop();
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        idle();
     }

    private void slideTest ()
        throws InterruptedException
    {
        robot.servoSlide.setInitialPosition();
        idle();

        long timeStamp;
        back:
        for (int j = 0; j < 2; j++) {
            float p = 0.35f;

            for (int i = 0; i < 2; i++) {
                Delayed delay = new Delayed();

                while (opModeIsActive()) {
                    timeStamp = System.currentTimeMillis();
                    robot.slideLeft.setPower(p);
                    robot.slideRight.setPower(p);

                    telemetry.addData("Left",
                                      numberFormatter.format(robot.slideLeft.getCurrentPosition()));
                    telemetry.addData("Right",
                                      numberFormatter.format(robot.slideRight.getCurrentPosition()));
                    telemetry.addData("Stop", "Back");

                    if (p > 0 && robot.switchSlideUp.isTouch()) {
                        if (delay.isDelayed(timeStamp)) {
                            delay.reset();
                            break;
                        }
                    } else if (p < 0 && robot.switchSlideDown.isTouch()) {
                        if (delay.isDelayed(timeStamp)) {
                            robot.slideLeft.resetPosition();
                            robot.slideRight.resetPosition();
                            delay.reset();
                            break;
                        }
                    }
                    if (gamepad1.back && Button.BTN_BACK.canPress(timeStamp)) {
                        break back;
                    }

                    idle();
                }
                p = -p;

                robot.slideLeft.setPower(0);
                robot.slideRight.setPower(0);
                idle();

                if (j == 0 && i == 0) {
                    // wiggle bucket
                    for (ServoTestInfo servoTestInfo: bucketServoTestInfos) {
                        servoTest(servoTestInfo);
                    }
                }
                else if (j == 1) {
                    // pop
                    robot.servoSlide.setPosition(RobotConstants.SERVO_SLIDE_END);
                    idle();
                }
            } // i
        } // j

        robot.slideLeft.setPower(0);
        robot.slideRight.setPower(0);
        idle();

        WaitLinear lp = new WaitLinear(this);
        lp.waitMillis(1500,
                      new WaitLinear.WakeUp() {
                          public boolean isWakeUp() {
                              long timeStamp = System.currentTimeMillis();
                              if (gamepad1.back && Button.BTN_BACK.canPress(timeStamp)) {
                                  telemetry.addData("Stop", "Back");
                                  return true;
                              } else {
                                  return false;
                              }
                          }
                      });

        robot.servoSlide.setInitialPosition();
        robot.slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        idle();
    }

    private void rollerTest ()
        throws InterruptedException
    {
        float[] p = new float[] {0.75f, -0.50f};

        back:
        for (int i = 0; i < 2; i++) {
            idle();

            long startTimestamp = System.currentTimeMillis();
            long timeStamp = startTimestamp;

            while (opModeIsActive() &&
                timeStamp - startTimestamp < 2000) {

                robot.roller.setPower(p[i]);
                if (gamepad1.back && Button.BTN_BACK.canPress(timeStamp)) {
                    break back;
                }

                telemetry.addData("Test", "Roller");
                telemetry.addData("Stop", "Back");

                idle();
                timeStamp = System.currentTimeMillis();
            }

            robot.roller.setPower(0);
            idle();
        }

        robot.roller.setPower(0);
        robot.roller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
            else if (gamepad1.back && Button.BTN_BACK.canPress(timeStamp)) {
                return;
            }
            else if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {
                servoTest(servoTestInfos[servoTestCounter]);
            }

            telemetry.addData("Select", "Next:RightTrigger Prev:LeftTrigger");
            telemetry.addData("Servo", servoTestInfos[servoTestCounter].servo);
            telemetry.addData("Confirm", "Start");
            telemetry.addData("Stop", "Back");

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

            while (opModeIsActive() &&
                timeStamp - startTimestamp < timeout) {
                servoTestInfo.servo.setPosition(position);

                if (gamepad1.back && Button.BTN_BACK.canPress(timeStamp)) {
                    return;
                }

                telemetry.addData("Position", Float.toString(servoTestInfo.servo.getPosition()));
                telemetry.addData("Servo", servoTestInfo.servo);
                telemetry.addData("Stop", "Back");

                idle();
                timeStamp = System.currentTimeMillis();
            }
        }
    }

    private void ledTest (List<HwLed> ledList)
        throws InterruptedException
    {
        for (HwLed led: ledList) {

            boolean on = true;
            for (int i = 0; i < 2; i++) {
                long startTimestamp = System.currentTimeMillis();
                long timeStamp = startTimestamp;

                while (opModeIsActive() &&
                    timeStamp - startTimestamp < 1500) {

                    if (gamepad1.back && Button.BTN_BACK.canPress(timeStamp)) {
                        return;
                    }

                    led.set(on);
                    led.draw();
                    telemetry.addData("LED", led);
                    telemetry.addData("Stop", "Back");

                    idle();
                    timeStamp = System.currentTimeMillis();
                }
                on = false;
            }
        }
    }

    private void gyroTest()
        throws InterruptedException
    {
        idle();

        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {

                robot.gyroSensor.resetHeading();
                idle();
            }
            else if (gamepad1.back && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }

            telemetry.addData("Heading",
                              Integer.toString(robot.gyroSensor.getHeading()));
            telemetry.addData("Reset", "Start");
            telemetry.addData("Stop", "Back");

            idle();
        }
        idle();
    }

    private void colorTest()
        throws InterruptedException
    {
        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            if (gamepad1.back && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }

            for (HwColorSensor colorSensor : colorSensors) {
                telemetry.addData(colorSensor.getId() + "-ARGB",
                                  "" + 8 * colorSensor.alpha() +
                                      ':' + 8 * colorSensor.red() +
                                      ':' + 8 * colorSensor.green() +
                                      ':' + 8 * colorSensor.blue());
                // hsvValues is an array that will hold the hue, saturation, and value information.
                float hsvValues[] = {0F,0F,0F};
                // convert the RGB values to HSV values.
                Color.RGBToHSV(colorSensor.red() * 8,
                               colorSensor.green() * 8,
                               colorSensor.blue() * 8,
                               hsvValues);
                telemetry.addData(colorSensor.getId() + "-HSV",
                                  "" + ((int)hsvValues[0]) +
                                      ':' + ((int)(hsvValues[1]*100)) +
                                      ':' + ((int)(hsvValues[2]*100)));

                String color = "";
                if (colorSensor.isRed()) {
                    color = "Red";
                }
                else if (colorSensor.isBlue()) {
                    color = "Blue";
                }
                else if (colorSensor.isGreen()) {
                    color = "Green";
                }
                telemetry.addData(colorSensor.getId() + "-Color", color);
            } // for colorSensor

            Boolean isLeftRed = robot.isLeftRed();
            String leftColor = "";
            if (isLeftRed != null) {
                leftColor = isLeftRed ? "Red" : "Blue";
            }
            telemetry.addData("LeftColor", leftColor);
/*
            String error = "";
            if (robot.colorLeftIn.isBlue() &&
                !robot.colorRightIn.isRed() &&
                !robot.colorRightOut.isRed() &&
                !robot.colorRightIn.isBlue() &&
                !robot.colorRightOut.isBlue()) {
                error = "Error";
            }
            else if (robot.colorLeftIn.isRed() &&
                !robot.colorRightIn.isBlue() &&
                !robot.colorRightOut.isBlue() &&
                !robot.colorRightIn.isRed() &&
                !robot.colorRightOut.isRed()) {
                error = "Error";
            }
            else if (robot.colorRightIn.isBlue() &&
                !robot.colorLeftIn.isRed() &&
                !robot.colorLeftOut.isRed() &&
                !robot.colorLeftIn.isBlue() &&
                !robot.colorLeftOut.isBlue()) {
                error = "Error";
            }
            else if (robot.colorRightIn.isRed() &&
                !robot.colorLeftIn.isBlue() &&
                !robot.colorLeftOut.isBlue() &&
                !robot.colorLeftIn.isRed() &&
                !robot.colorLeftOut.isRed()) {
                error = "Error";
            }
            telemetry.addData("Error", error);
*/
            telemetry.addData("Stop", "Back");
            idle();
        }
    }

    private void beaconTest ()
        throws InterruptedException
    {
        SelectLinear sl = new SelectLinear(this);
        boolean isRed = sl.selectAlliance();
        WaitLinear lp = new WaitLinear(this);
        long startTimestamp = System.currentTimeMillis();
        Boolean bLeftRed = robot.isLeftRed();
        if (bLeftRed != null) {
            float servoBeaconPosition;
            if (bLeftRed) {
                servoBeaconPosition = isRed ? RobotConstants.SERVO_BEACON_LEFT : RobotConstants.SERVO_BEACON_RIGHT;
            }
            else {
                servoBeaconPosition = isRed ? RobotConstants.SERVO_BEACON_RIGHT : RobotConstants.SERVO_BEACON_LEFT;
            }
            telemetry.addData("Left is Red?", bLeftRed);
            robot.servoBeacon.setPosition(servoBeaconPosition);
        }
        else {
            telemetry.addData("Left is Red?", "Unable to detect color");
        }
        lp.waitMillis(4000,
                      startTimestamp);

        robot.servoBeacon.setPosition(RobotConstants.SERVO_BEACON_MID);
    }

    private class ServoTestInfo {
        private HwServo servo;
        private int timeScale = 1;
        private float[] positions;

        /**
         *
         * @param servo
         * @param positions servo positions other than the initial position
         */
        private ServoTestInfo(HwServo servo,
                              float... positions) {
            this.servo = servo;
            this.positions = positions;
        }

        /**
         *
         * @param servo
         * @param positions servo positions other than the initial position
         */
        private ServoTestInfo(HwServo servo,
                              int timeScale,
                              float... positions) {
            this(servo, positions);
            this.timeScale = timeScale;
        }
    }
}
