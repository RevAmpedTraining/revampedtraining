package org.firstinspires.ftc.teamcode;

import com.revAmped.components.Button;
import com.revAmped.components.HwLed;
import com.revAmped.components.HwServo;
import com.revAmped.components.HwSonarAnalog;
import com.revAmped.components.HwSwitch;
import com.revAmped.components.TurnType;
import com.revAmped.config.RobotConstants;
import com.revAmped.config.RobotConstants.COLOR_SENSOR;
import com.revAmped.linear.components.RobotLinear;
import com.revAmped.linear.components.SwerveDriveLinear;
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

    private final static int MIN_SERVO_TICK = 1;
    private final static DecimalFormat numberFormatter = new DecimalFormat("######");

    private RobotLinear robot;
    private SwerveDriveLinear drive;

    private ServoTestInfo[] servoTestInfos;

    private int servoTestCounter = 0;

    private int servoCalibrateCounter = 0;

    /**
     * enumeration for the tests to be run
     */
    public enum ETest {
        NONE,
        SWITCH,
        ENCODER,
        DRIVE,
        SERVO_CALIBRATE,
        SERVO,
        SPINNER,
        SLIDE,
        ROLLER,
        LED,
        GYRO,
        COLOR,
        SONAR;

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
        drive = robot.getSwerveDriveLinear();
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoTestInfos = new ServoTestInfo[] {
                new ServoTestInfo(robot.servoLeftFront,
                                  RobotConstants.SERVO_LEFTFRONT_END),
                new ServoTestInfo(robot.servoLeftBack,
                                  RobotConstants.SERVO_LEFTBACK_END),
                new ServoTestInfo(robot.servoRightFront,
                                  RobotConstants.SERVO_RIGHTFRONT_END),
                new ServoTestInfo(robot.servoRightBack,
                                  RobotConstants.SERVO_RIGHTBACK_END),
                new ServoTestInfo(robot.servoTrigger,
                                  RobotConstants.SERVO_TRIGGER_OUT),
                new ServoTestInfo(robot.servoBeacon,
                                  2,
                                  RobotConstants.SERVO_BEACON_OUT,
                                  RobotConstants.SERVO_BEACON_IN),
                new ServoTestInfo(robot.servoCapBall,
                                  RobotConstants.SERVO_CAP_BALL_PUSH,
                                  RobotConstants.SERVO_CAP_BALL_HOLD),
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
                    case SPINNER:
                        spinnerTest();
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
                        ledTest();
                        break;
                    case GYRO:
                        gyroTest();
                        break;
                    case COLOR:
                        colorTest();
                        break;
                    case SONAR:
                        sonarTest();
                        break;
                    case NONE:
                    default:
                        break;
                }
            }

            telemetry.update();
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

            for (HwSwitch s : robot.switchs) {
                telemetry.addData(s.toString(), Boolean.toString(s.isTouch()));
            }

            telemetry.addData("Stop", "Back");

            telemetry.update();
            idle();
        }
    }

    /**
     * shows the encoder values
     * @throws InterruptedException
     */
    private void encoderTest ()
        throws InterruptedException
    {
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {

                drive.resetPosition();
                robot.spinnerLeft.resetPosition();
                robot.spinnerRight.resetPosition();
                idle();
            }
            else if (gamepad1.back && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }

            telemetry.addData("Front",
                              "Left:" + numberFormatter.format(robot.driveLeftFront.getCurrentPosition()) +
                                  " Right:" + numberFormatter.format(robot.driveRightFront.getCurrentPosition()));
            telemetry.addData("Back",
                              "Left:" + numberFormatter.format(robot.driveLeftBack.getCurrentPosition()) +
                                  " Right:" + numberFormatter.format(robot.driveRightBack.getCurrentPosition()));
            telemetry.addData("Spinner",
                              "Left:" + numberFormatter.format(robot.spinnerLeft.getCurrentPosition()) +
                                  " Right:" + numberFormatter.format(robot.spinnerRight.getCurrentPosition()));
            telemetry.addData("Reset", "Start");
            telemetry.addData("Stop", "Back");

            telemetry.update();
            idle();
        }

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        idle();
    }

    /**
     * calibration for each servo
     * @param servoCalibrateList servos to be tested
     * @throws InterruptedException
     */
    private void servoCalibrate(List<HwServo> servoCalibrateList)
        throws InterruptedException
    {
        int posJoy1 = (int)(servoCalibrateList.get(servoCalibrateCounter).getPosition() * 255f);

        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            if(gamepad1.right_trigger > 0.9 && Button.BTN_NEXT.canPress(timeStamp)) {
                servoCalibrateCounter++;
                if(servoCalibrateCounter >= servoCalibrateList.size()){
                    servoCalibrateCounter = 0;
                }
                posJoy1 = (int)(servoCalibrateList.get(servoCalibrateCounter).getPosition() * 255f);
            } else if(gamepad1.left_trigger > 0.9 && Button.BTN_PREV.canPress(timeStamp)) {
                servoCalibrateCounter--;
                if(servoCalibrateCounter < 0){
                    servoCalibrateCounter = servoCalibrateList.size() - 1;
                }
                posJoy1 = (int)(servoCalibrateList.get(servoCalibrateCounter).getPosition() * 255f);
            }
            else if (gamepad1.back && Button.BTN_BACK.canPress(timeStamp)) {
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
            servoCalibrateList.get(servoCalibrateCounter).setPosition(posJoy1/255f);

            telemetry.addData("Adjust", "+: B -: X Max: Y Min: A Mid: RStick");
            telemetry.addData("Position", Integer.toString(posJoy1));
            telemetry.addData("Servo", servoCalibrateList.get(servoCalibrateCounter));
            telemetry.addData("Select", "Next: RightTrigger Prev: LeftTrigger");
            telemetry.addData("Stop", "Back");

            telemetry.update();
            idle();
        }
    }

    /**
     * tests the swerve drive motors and servos
     * @throws InterruptedException
     */
    private void driveTest ()
        throws InterruptedException
    {
        float[][] p = new float[][]{
            {0.25f, 0.25f},
            {-0.25f, -0.25f},
            {0.25f, -0.25f},
            {-0.25f, 0.25f},
        };

        TurnType[] t = new TurnType[]{
                TurnType.FORWARD,
                TurnType.STRAFE,
                TurnType.TURN_REGULAR,
        };

        back:
        for (int j = 0; j < t.length; j++) {
            for (int i = 0; i < p.length; i++) {
                drive.resetPosition();

                long startTimestamp = System.currentTimeMillis();
                long timeStamp = startTimestamp;

                while (opModeIsActive() &&
                        timeStamp - startTimestamp < 5000) {

                    drive.setTurn(t[j]);
                    drive.setPower(p[i][0],
                                   p[i][1],
                                   t[j]);

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

                    telemetry.update();
                    idle();
                    timeStamp = System.currentTimeMillis();
                }

                drive.stop();
            }
        }

        drive.stop();
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        idle();
     }

    /**
     * runs the shooter
     * @throws InterruptedException
     */
    private void spinnerTest ()
            throws InterruptedException
    {
        final float powerSpinner = 0.2f;
        long startTimestamp = System.currentTimeMillis();
        long timeStamp = startTimestamp;

        while (opModeIsActive() &&
                timeStamp - startTimestamp < 5000) {

            robot.spinnerLeft.setPower(powerSpinner);
            robot.spinnerRight.setPower(powerSpinner);
            if (gamepad1.back && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }

            telemetry.addData("Test", "Spinner");
            telemetry.addData("Stop", "Back");
            telemetry.addData("Left", robot.spinnerLeft.getCurrentPosition());
            telemetry.addData("Right", robot.spinnerRight.getCurrentPosition());
            telemetry.update();
            idle();
            timeStamp = System.currentTimeMillis();
        }

        robot.spinnerLeft.setPower(0f);
        robot.spinnerRight.setPower(0f);
    }

    /**
     * moves the slide up and down
     * @throws InterruptedException
     */
    private void slideTest ()
        throws InterruptedException
    {
        long timeStamp = System.currentTimeMillis();
        WaitLinear lp = new WaitLinear(this);
        final float slidePower = 0.75f*RobotConstants.POWER_SLIDE;

        while (opModeIsActive() && !robot.switchSlideUp.isTouch()) {
            robot.slide.setPower(slidePower);
            if (gamepad1.back && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }

            telemetry.addData("Test", "Slide");
            telemetry.addData("Stop", "Back");
            telemetry.addData("Switch Up", robot.switchSlideUp.isTouch());
            telemetry.addData("Switch Down", robot.switchSlideDown.isTouch());
            telemetry.update();
            idle();
            timeStamp = System.currentTimeMillis();
        }

        for (float servoPosition : new float[]{RobotConstants.SERVO_CAP_BALL_PUSH,
                RobotConstants.SERVO_CAP_BALL_HOLD}) {
            long startTimestamp = System.currentTimeMillis();
            timeStamp = startTimestamp;
            while (opModeIsActive() &&
                    timeStamp - startTimestamp < 1500) {
                robot.servoCapBall.setPosition(servoPosition);

                if (robot.switchSlideUp.isTouch()) {
                    robot.slide.setPower(0f);
                } else {
                    robot.slide.setPower(slidePower);
                }

                if (gamepad1.back && Button.BTN_BACK.canPress(timeStamp)) {
                    break;
                }

                telemetry.addData("Test", "Slide");
                telemetry.addData("Stop", "Back");
                telemetry.addData("Switch Up", robot.switchSlideUp.isTouch());
                telemetry.addData("Switch Down", robot.switchSlideDown.isTouch());
                telemetry.update();
                idle();
                timeStamp = System.currentTimeMillis();
            }
        }

        robot.servoCapBall.setPosition(RobotConstants.SERVO_CAP_BALL_OPEN);

        while (opModeIsActive() && !robot.switchSlideDown.isTouch()) {
            robot.slide.setPower(-slidePower);
            if (gamepad1.back && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }

            telemetry.addData("Test", "Slide");
            telemetry.addData("Stop", "Back");
            telemetry.addData("Switch Up", robot.switchSlideUp.isTouch());
            telemetry.addData("Switch Down", robot.switchSlideDown.isTouch());
            telemetry.update();
            idle();
            timeStamp = System.currentTimeMillis();
        }

        robot.slide.setPower(0f);
        idle();
    }

    /**
     * runs the rollers in and out
     * @throws InterruptedException
     */
    private void rollerTest ()
        throws InterruptedException
    {
        float[] p = new float[] {0.75f, -0.50f};

        back:
        for (int i = 0; i < 2; i++) {

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

                telemetry.update();
                idle();
                timeStamp = System.currentTimeMillis();
            }

            robot.roller.setPower(0);
            idle();
        }

        robot.roller.setPower(0);
        idle();
    }

    /**
     * tests each of the servos
     * @param servoTestInfos the servos to be tested
     * @throws InterruptedException
     */
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

            telemetry.update();
            idle();
        }
    }

    /**
     * tests each of the servos
     * @param servoTestInfo the servos to be tested
     * @throws InterruptedException
     */
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

                telemetry.update();
                idle();
                timeStamp = System.currentTimeMillis();
            }
        }
    }

    /**
     * turns on each of the 5 LEDs
     * @throws InterruptedException
     */
    private void ledTest ()
        throws InterruptedException
    {
        final HwLed.ELedStatus[] ledStatuses = new HwLed.ELedStatus[] {
                HwLed.ELedStatus.LED_ON,
                HwLed.ELedStatus.LED_BLINK,
                HwLed.ELedStatus.LED_OFF};
        for (HwLed led: robot.leds) {

            for (int i = 0; i < ledStatuses.length; i++) {
                long startTimestamp = System.currentTimeMillis();
                long timeStamp = startTimestamp;

                while (opModeIsActive() &&
                    timeStamp - startTimestamp < 2000) {

                    if (gamepad1.back && Button.BTN_BACK.canPress(timeStamp)) {
                        return;
                    }

                    led.set(ledStatuses[i]);
                    led.draw();
                    telemetry.addData("LED", led + " " + led.getChannel() + " " + ledStatuses[i]);
                    telemetry.addData("Stop", "Back");

                    telemetry.update();
                    idle();
                    timeStamp = System.currentTimeMillis();
                }
            }
        }
    }

    /**
     * shows IMU headings
     * @throws InterruptedException
     */
    private void gyroTest()
        throws InterruptedException
    {
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
                              Float.toString(robot.gyroSensor.getHeading()));
            telemetry.addData("Reset", "Start");
            telemetry.addData("Stop", "Back");

            telemetry.update();
            idle();
        }
        idle();
    }

    /**
     * shows color sensor readings
     * @throws InterruptedException
     */
    private void colorTest()
        throws InterruptedException
    {
        COLOR_SENSOR[] colorSensors = robot.colorSensor.getColorSensors();

        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            if (gamepad1.back && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }

            for (COLOR_SENSOR colorSensor : colorSensors) {

                int[] crgb = robot.colorSensor.getCRGB(colorSensor);

                telemetry.addLine("Sensor " + colorSensor);
                telemetry.addData("CRGB", "%5d %5d %5d %5d",
                                  crgb[0], crgb[1], crgb[2], crgb[3]);

                int lux = robot.colorSensor.luminance(crgb);
                 int ct = robot.colorSensor.colorTemperature(crgb);
                telemetry.addData("LUX:CT",
                                  "" + lux + ':' + ct);

                // hsvValues is an array that will hold the hue, saturation, and value information.
                float hsvValues[] = robot.colorSensor.hsvValues(crgb);
                telemetry.addData("HSV",
                                  "" + ((int)hsvValues[0]) +
                                      ':' + ((int)(hsvValues[1]*100)) +
                                      ':' + ((int)(hsvValues[2]*100)));

                String color = "";
                if (robot.colorSensor.isRed(crgb)) {
                    color = "Red";
                }
                else if (robot.colorSensor.isBlue(crgb)) {
                    color = "Blue";
                }
                telemetry.addData("Color", color);
            } // for colorSensor

            int[] crgbLF = robot.colorSensor.getCRGB(COLOR_SENSOR.LEFT_FRONT);
            int[] crgbLB = robot.colorSensor.getCRGB(COLOR_SENSOR.LEFT_BACK);
            int[] crgbRF = robot.colorSensor.getCRGB(COLOR_SENSOR.RIGHT_FRONT);
            int[] crgbRB = robot.colorSensor.getCRGB(COLOR_SENSOR.RIGHT_BACK);

            Boolean leftRed = robot.colorSensor.isRed(crgbLF,
                                                      crgbLB);
            if (Boolean.TRUE.equals(leftRed)) {
                telemetry.addData("Left", "Red");
            }
            else if (Boolean.FALSE.equals(leftRed)) {
                telemetry.addData("Left", "Blue");
            }
            else {
                telemetry.addData("Left", "None");
            }

            Boolean rightRed = robot.colorSensor.isRed(crgbRF,
                                                       crgbRB);
            if (Boolean.TRUE.equals(rightRed)) {
                telemetry.addData("Right", "Red");
            }
            else if (Boolean.FALSE.equals(rightRed)) {
                telemetry.addData("Right", "Blue");
            }
            else {
                telemetry.addData("Right", "None");
            }

            telemetry.addData("Stop", "Back");
            telemetry.update();
            idle();
        }
    }

    /**
     * shows ultrasonic sensor values
     * @throws InterruptedException
     */
    private void sonarTest ()
            throws InterruptedException {
        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            if (gamepad1.back && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }

            for (HwSonarAnalog sonar : robot.sonars) {
                float distance = sonar.getDistance();
                telemetry.addLine("Sensor " + sonar);
                telemetry.addData("Distance", Float.toString(distance));
            }

            telemetry.addData("Stop", "Back");
            telemetry.update();
            idle();
        }
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
