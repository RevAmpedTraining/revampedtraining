package org.firstinspires.ftc.teamcode.RevAmpedGame;

import com.revAmped.components.Button;
import com.revAmped.components.HwDistanceSensor;
import com.revAmped.components.HwLed;
import com.revAmped.components.HwNormalizedColorSensor;
import com.revAmped.components.HwServo;
import com.revAmped.components.HwSonarAnalog;
import com.revAmped.components.HwSwitch;
import com.revAmped.components.RobotRevAmped;
import com.revAmped.components.TurnType;
import com.revAmped.config.RobotOverchargedConstants;
import com.revAmped.config.RobotRevAmpedConstants;
import com.revAmped.linear.components.MecanumDriveLinear;
import com.revAmped.linear.components.RobotRevAmpedLinear2;
import com.revAmped.linear.components.RobotRevAmpedLinear;
import com.revAmped.linear.util.SelectLinear;
import com.revAmped.linear.util.WaitLinear;
import com.revAmped.sensors.RevColorDistanceSensor;
import com.revAmped.test.ServoTestInfo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.text.DecimalFormat;
import java.util.List;

//Vuforia test
//=======
//Vuforia test
//=======

/**
 * RobotRevAmped Tester
 */
@TeleOp(name="Tester RevAmped", group="Test")
public class TesterRevAmped
        extends LinearOpMode {

    private final static float MIN_SERVO_TICK = 1;
    private final static DecimalFormat numberFormatter = new DecimalFormat("######");

    private RobotRevAmpedLinear2 robot;
    private MecanumDriveLinear drive;

    private ServoTestInfo[] servoTestInfos;

    private int servoTestCounter = 0;

    private int servoCalibrateCounter = 0;

    //vuforia test variables
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
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
        SLIDE,
        SLIDE_RELIC,
        SWEEPER,
        LED,
        GYRO,
        TURN_180_DEGREE,
        VUFORIA,
        REV_COLOR_SENSOR,
        JEWEL_SERVO,
        RANGE_SENSOR;

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
        robot = new RobotRevAmpedLinear2(this);
        drive = robot.getMecanumDriveLinear();

        servoTestInfos = new ServoTestInfo[] {
                new ServoTestInfo(robot.servoContainer,
                        RobotRevAmpedConstants.SERVO_CONTAINER_UP),
                new ServoTestInfo(robot.servoDoorRight,
                                  RobotRevAmpedConstants.SERVO_DOOR_IN,
                                  RobotRevAmpedConstants.SERVO_DOOR_OUT),
                new ServoTestInfo(robot.servoJewel,
                                  RobotRevAmpedConstants.SERVO_JEWEL_OUT),
                new ServoTestInfo(robot.servoJewelHit,
                                  RobotRevAmpedConstants.SERVO_HIT_RIGHT,
                                  RobotRevAmpedConstants.SERVO_HIT_LEFT),
                new ServoTestInfo(robot.servoDumperClaw,
                                 RobotRevAmpedConstants.SERVO_DUMPER_CLAW_IN),
                new ServoTestInfo(robot.servoRelicClaw,
                                  2,
                                  RobotRevAmpedConstants.SERVO_RELIC_CLAW_GRAB),
                new ServoTestInfo(robot.servoRelicElbow,
                                  12,
                                  RobotRevAmpedConstants.SERVO_RELIC_ELBOW_UP),
                new ServoTestInfo(robot.servoStick,
                        RobotRevAmpedConstants.SERVO_STICK_OUT)

        };

        int testCounter = 0;
        ETest currentTest = ETest.NONE;

        //telemetry.addData("Waiting", "LinearTester");
        //telemetry.update();
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
                    case SLIDE_RELIC:
                        slideRelicTest();
                        break;
                    case SWEEPER:
                        sweeperTest();
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
                    case TURN_180_DEGREE:
                        turnWithGyroInfo(180.0f);
                        break;
                    case VUFORIA:
                        vuforiaTest();
                        break;
                    case REV_COLOR_SENSOR:
                        revColorSensorTest();
                        break;
                    case JEWEL_SERVO:
                        jewelServo();
                        break;
                    case RANGE_SENSOR:
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

    private void encoderTest ()
            throws InterruptedException
    {
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.motorRelicSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {

                drive.resetPosition();
                robot.slide.resetPosition();
                robot.motorRelicSlide.resetPosition();
                idle();
            }
            else if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }

            telemetry.addData("Front",
                    "Left:" + numberFormatter.format(robot.driveLeftFront.getCurrentPosition()) +
                            " Right:" + numberFormatter.format(robot.driveRightFront.getCurrentPosition()));
            telemetry.addData("Back",
                    "Left:" + numberFormatter.format(robot.driveLeftBack.getCurrentPosition()) +
                            " Right:" + numberFormatter.format(robot.driveRightBack.getCurrentPosition()));
            telemetry.addData("Sweeper",
                              "Left:" + numberFormatter.format(robot.sweeperLeft.getCurrentPosition()) +
                                      " Right:" + numberFormatter.format(robot.sweeperRight.getCurrentPosition()));
            telemetry.addData("Slide",
                    "Slide:" + numberFormatter.format(robot.slide.getCurrentPosition()));
            telemetry.addData("Relic",
                              "Slide:" + numberFormatter.format(robot.motorRelicSlide.getCurrentPosition()));
            telemetry.addData("Reset", "Start");
            telemetry.addData("Stop", "Back");
            telemetry.update();
            idle();
        }

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        idle();
    }

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
            else if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
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

    private void driveTest ()
            throws InterruptedException
    {
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

    private void slideTest ()
            throws InterruptedException
    {
        long timeStamp = System.currentTimeMillis();
        final float slidePower = 0.75f* RobotOverchargedConstants.POWER_SLIDE;
        robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_IN+40f/255f);
        for (int i = 0; i < 2; i++) {
            while (opModeIsActive() && !robot.switchSlideUp.isTouch()) {
                robot.slide.setPower(slidePower);
                if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                    break;
                }

                telemetry.addData("Test", "Slide");
                telemetry.addData("Stop", "Back");
                telemetry.addData("Switch Up", robot.switchSlideUp.isTouch());
                telemetry.addData("Switch Down", robot.switchSlideDown.isTouch());
                telemetry.addData("Slide",
                                  numberFormatter.format(robot.slide.getCurrentPosition()));
                telemetry.update();
                idle();
                timeStamp = System.currentTimeMillis();
            }

            robot.slide.stop();
            while (opModeIsActive() && !robot.switchSlideDown.isTouch()) {
                robot.slide.setPower(-slidePower);
                if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                    break;
                }

                telemetry.addData("Test", "Slide");
                telemetry.addData("Stop", "Back");
                telemetry.addData("Switch Up", robot.switchSlideUp.isTouch());
                telemetry.addData("Switch Down", robot.switchSlideDown.isTouch());
                telemetry.addData("Slide",
                                  numberFormatter.format(robot.slide.getCurrentPosition()));
                telemetry.update();
                idle();
                timeStamp = System.currentTimeMillis();
            }

            robot.slide.stop();
            idle();
        }
    }

    private void slideRelicTest ()
            throws InterruptedException
    {
        long timeStamp = System.currentTimeMillis();
        final float slidePower = 0.75f* RobotOverchargedConstants.POWER_SLIDE;
        WaitLinear lp = new WaitLinear(this);

        robot.servoRelicElbow.setPosition(RobotRevAmpedConstants.SERVO_RELIC_ELBOW_UP);
        lp.waitMillis(2000);

        for (int i = 0; i < 2; i++) {
            long startTime = System.currentTimeMillis();
            while (opModeIsActive() && (System.currentTimeMillis() - startTime < 2000)) {
                robot.motorRelicSlide.setPower(slidePower);
                if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                    break;
                }

                telemetry.addData("Test", "Relic");
                telemetry.addData("Stop", "Back");
                telemetry.addData("Switch In", robot.switchRelicSlideIn.isTouch());
                telemetry.addData("Relic",
                                  numberFormatter.format(robot.motorRelicSlide.getCurrentPosition()));
                telemetry.update();
                idle();
                timeStamp = System.currentTimeMillis();
            }

            robot.motorRelicSlide.setPower(0);
            robot.servoRelicClaw.setPosition(RobotRevAmpedConstants.SERVO_RELIC_CLAW_OPEN);
            lp.waitMillis(1000);
            robot.servoRelicClaw.setPosition(RobotRevAmpedConstants.SERVO_RELIC_CLAW_DROP);
            lp.waitMillis(1000);
            robot.servoRelicClaw.setPosition(RobotRevAmpedConstants.SERVO_RELIC_CLAW_WIDEOPEN);
            lp.waitMillis(1000);
            robot.servoRelicClaw.setPosition(RobotRevAmpedConstants.SERVO_RELIC_CLAW_GRAB);
            lp.waitMillis(1000);
            robot.servoRelicClaw.setPosition(RobotRevAmpedConstants.SERVO_RELIC_CLAW_REST);
            lp.waitMillis(1000);

            while (opModeIsActive() && !robot.switchRelicSlideIn.isTouch()) {
                robot.motorRelicSlide.setPower(-slidePower);
                if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                    break;
                }

                telemetry.addData("Test", "Relic");
                telemetry.addData("Stop", "Back");
                telemetry.addData("Switch In", robot.switchRelicSlideIn.isTouch());
                telemetry.addData("Relic",
                                  numberFormatter.format(robot.motorRelicSlide.getCurrentPosition()));
                telemetry.update();
                idle();
                timeStamp = System.currentTimeMillis();
            }

            robot.motorRelicSlide.setPower(0);
            idle();
        }

        robot.servoRelicElbow.setPosition(RobotRevAmpedConstants.SERVO_RELIC_ELBOW_REST);
        robot.servoRelicClaw.setPosition(RobotRevAmpedConstants.SERVO_RELIC_CLAW_REST);
        lp.waitMillis(2000);
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


    /**
     * runs the sweepers in and out
     * @throws InterruptedException
     */
    private void sweeperTest()
            throws InterruptedException
    {
        float[] p = new float[] {0.75f, -0.50f};

        back:
        for (int i = 0; i < 2; i++) {

            long startTimestamp = System.currentTimeMillis();
            long timeStamp = startTimestamp;

            while (opModeIsActive() &&
                    timeStamp - startTimestamp < 2000) {

                robot.sweeperLeft.setPower(p[i]);
                robot.sweeperRight.setPower(p[i]);
                if (gamepad1.back && Button.BTN_BACK.canPress(timeStamp)) {
                    break back;
                }

                telemetry.addData("Test", "Sweeper");
                telemetry.addData("Stop", "Back");

                telemetry.update();
                idle();
                timeStamp = System.currentTimeMillis();
            }

            robot.sweeperLeft.setPower(0);
            idle();
        }

        robot.sweeperLeft.setPower(0);
        robot.sweeperRight.setPower(0);
        idle();
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
            throws InterruptedException
    {
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
    private void turnWithGyroInfo(float angle)
    {
        robot.gyroSensor.resetHeading();  // get the current rawheading and set it to base
        float curYaw = robot.gyroSensor.getHeading();  // use current heading subtracting base
        long timeStamp = System.currentTimeMillis();

        // if the gamepad1's stick_x button is not held, then it will exit, and stop the robot.
        while (opModeIsActive() && Math.abs( curYaw)  < angle && (gamepad1.left_stick_x > 0.5 || gamepad1.right_stick_x > 0.5))
        {
            if (gamepad1.left_stick_x > 0.5) {
                drive.setPower(-1.0f, 0.0f, TurnType.TURN_LEFT_PIVOT);
            }
            else if (gamepad1.right_stick_x > 0.5){
                drive.setPower(0.0f, 1.0f, TurnType.TURN_RIGHT_PIVOT);
            }
            curYaw = robot.gyroSensor.getHeading();
            telemetry.addData("YAW=", Float.toString(curYaw));
            telemetry.addData("Target=", Float.toString(angle));
            telemetry.addData("Drive: ", "Turning");

            timeStamp =  System.currentTimeMillis();
            telemetry.update();

            try {
                Thread.sleep(100,0);
            }catch(InterruptedException e) {};

        }
        drive.stop();
        telemetry.addData("Drive: ", "Stopped");
        telemetry.update();
        idle();
    }



    private String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    private void revColorSensorTest()
    {
        while (opModeIsActive())
        {
            long timeStamp = System.currentTimeMillis();

            if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }
            robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_JEWEL_OUT);
            RevColorDistanceSensor.COLORTYPE jewelcolor = robot.revColorDistanceSensor.getColor();
            String strColor;
            if (jewelcolor == RevColorDistanceSensor.COLORTYPE.RED)
            {
                strColor = "RED";
            }
            else if (jewelcolor == RevColorDistanceSensor.COLORTYPE.BLUE)
            {
                strColor = "BLUE";
            }
            else
                strColor = "NONE";

            telemetry.addData("JEWEL", strColor);
            // send the info back to driver station using telemetry function.

            //telemetry.addData("Distance (cm)",
            //       String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", robot.revColorDistanceSensor.getAlpha());
            telemetry.addData("Red  ", robot.revColorDistanceSensor.getRed());
            telemetry.addData("Green", robot.revColorDistanceSensor.getGreen());
            telemetry.addData("Blue ", robot.revColorDistanceSensor.getBlue());
            telemetry.addData("Hue", robot.revColorDistanceSensor.getHue());
            telemetry.update();
            idle();
        }
    }
    private void colorTest()
            throws InterruptedException
    {
        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }

            for (HwNormalizedColorSensor colorSensor : robot.colorSensors) {

                NormalizedRGBA nrgba = colorSensor.getNormalizedColors();

                telemetry.addLine("Sensor " + colorSensor);
                telemetry.addData("RGBC",
                                  nrgba.red + " " +
                                          nrgba.green + " " +
                                          nrgba.blue + " " +
                                          nrgba.alpha);

                float lux = colorSensor.luminance(nrgba);
                telemetry.addData("LUX:CT",
                                  "" + lux);

                // hsvValues is an array that will hold the hue, saturation, and value information.
                float hsvValues[] = colorSensor.hsvValues(nrgba);
                telemetry.addData("HSV",
                                  "" + ((int)hsvValues[0]) +
                                          ':' + ((int)(hsvValues[1]*100)) +
                                          ':' + ((int)(hsvValues[2]*100)));

                String color = "";
                if (colorSensor.isRed(hsvValues, 0)) {
                    color = "Red";
                }
                else if (colorSensor.isBlue(hsvValues, 0)) {
                    color = "Blue";
                }
                telemetry.addData("Color", color);
            } // for jewelColorSensor

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

            if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }

            for (HwDistanceSensor sonar : robot.sonars) {
                float distance = sonar.getDistance();
                telemetry.addLine("Sensor " + sonar);
                telemetry.addData("Distance", Float.toString(distance));
            }

            for (HwSonarAnalog sonar : robot.sonarAnalogs) {
                float distance = sonar.getDistance();
                telemetry.addLine("Sensor " + sonar);
                telemetry.addData("Distance", Float.toString(distance));
            }

            telemetry.addData("Stop", "Back");
            telemetry.update();
            idle();
        }
    }

    private void jewelServo()
            throws InterruptedException
    {
        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
                break;
            }
            SelectLinear sl = new SelectLinear(this);
            boolean isRed = sl.selectAlliance();
            telemetry.addData("Alliance", isRed ? "Red" : "Blue");
            RevColorDistanceSensor.COLORTYPE jewelcolor = robot.revColorDistanceSensor.getColor();
            String strColor;
            if (jewelcolor == RevColorDistanceSensor.COLORTYPE.RED)
            {
                strColor = "RED";

            }
            else if (jewelcolor == RevColorDistanceSensor.COLORTYPE.BLUE)
            {
                strColor = "BLUE";

            }
            else {
                strColor = "NONE";
            }


            telemetry.addData("JEWEL", strColor);
            telemetry.addData("Alpha", robot.revColorDistanceSensor.getAlpha());
            telemetry.addData("Red  ", robot.revColorDistanceSensor.getRed());
            telemetry.addData("Green", robot.revColorDistanceSensor.getGreen());
            telemetry.addData("Blue ", robot.revColorDistanceSensor.getBlue());
            telemetry.addData("Hue", robot.revColorDistanceSensor.getHue());
            telemetry.update();
            if ((jewelcolor == RevColorDistanceSensor.COLORTYPE.RED && isRed)
                    || (jewelcolor == RevColorDistanceSensor.COLORTYPE.BLUE && !isRed)) {
                robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_HIT_LEFT);
            }
            else if ((jewelcolor == RevColorDistanceSensor.COLORTYPE.BLUE && isRed)
                    || (jewelcolor == RevColorDistanceSensor.COLORTYPE.RED && !isRed)) {
                robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_HIT_RIGHT);

            } else {
                robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_DETECT);
            }
            telemetry.addData("Stop", "Back");
            telemetry.update();
            idle();
        }

    }


    private void vuforiaTest()
    {
                        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "ATZtEnj/////AAAAGT+CwB7OMkQJi55eZzs9StIAUPU0rdfyi9dSNXcaDLjU1P5Jl9flS5IgZCPhTFVEGGKIXukueuwprx3aNQeXujJjeXhMjEB3pcLLivaZY/EaRvZhuJPVPcAcF3RX7AVO4gPPpgvd4BVnSMlGqtoCUzAcOatW8xFTSPH5Z/IaAgw1OPQ9o6hv8DoxKcR8cVwO2NubyFRaGrl+8bDUCcuxt6PMcXF4wQ27fMRmsmc7LSRBdSOthfyn2O90chXpGrg42JqCPKKyctryWj30NEtTKnlIsZPkA3T5C4Lx/rxaFkg8fZ0MiGvh37WBuPVVh+njEHR0xJD7H0x4bZNBffsvK3aLmfDoVHVlLoBZ8DdhPcGg";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        //check opencv
        //if (OpenCVLoader.initDebug())
        //{
        telemetry.addData(">", "OpenCV lib loaded successfully.");
        //}
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();
        while (opModeIsActive()) {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);
                    telemetry.addData("X=", toString().valueOf(tX));
                    telemetry.addData("Y=", toString().valueOf(tY));
                    telemetry.addData("Z=", toString().valueOf(tZ));
                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                    telemetry.addData("rX=", toString().valueOf(rX));
                    telemetry.addData("rY=", toString().valueOf(rY));
                    telemetry.addData("rZ=", toString().valueOf(rZ));
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
            idle();
        }
    }


}
