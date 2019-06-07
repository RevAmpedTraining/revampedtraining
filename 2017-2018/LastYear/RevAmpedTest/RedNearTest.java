package org.firstinspires.ftc.teamcode.LastYear.RevAmpedTest;


import com.revAmped.components.Button;
import com.revAmped.components.HwSonarAnalog;
import com.revAmped.components.TurnType;
import com.revAmped.config.RobotRevAmpedConstants;
import com.revAmped.linear.components.MecanumDriveLinear;
import com.revAmped.linear.components.RobotRevAmpedLinear2;
import com.revAmped.linear.util.SelectLinear;
import com.revAmped.linear.util.WaitLinear;
import com.revAmped.sensors.RevColorDistanceSensor;
import com.revAmped.util.GlyphDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
/**
 * Autonomous Test for Red and Near
 * RevAmped 12808
 * Created Pi Day 2018
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RedNearTest2", group="Test")
public class RedNearTest
        extends LinearOpMode {
    //Revamped robot class
    private RobotRevAmpedLinear2 robot = null;
    //Revamped mecanum drive class
    private MecanumDriveLinear drive;
    //ultrasonic sensor for detecting glyphs
    private HwSonarAnalog sensor_sonar_front;

    private final static int MIN_FRONT_SPACING = -1;

    private GlyphDetector glyphCount;

    /**
     * autonomous opMode
     */
    @Override
    public void runOpMode()
            throws InterruptedException {
        try {
            // init
            robot = new RobotRevAmpedLinear2(this);
            drive = robot.getMecanumDriveLinear();
            glyphCount = new GlyphDetector(this.hardwareMap, "", "sensor_back", "");
            run();
        } finally {
            // shut down
            if (robot != null) {
                robot.close();
            }
        }
    }

    /**
     * autonomous function
     *
     * @throws InterruptedException
     */
    public void run()
            throws InterruptedException {
        /**
         * make the container flat to put a glyph inside and use the range
         * sensor to detect the distance to the wall, helping drive team with aligning the robot
         * at the start of autonomous
         */

        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
        float wallDistanceL;
        while (!Thread.currentThread().isInterrupted()) {
            long timeStamp = System.currentTimeMillis();
            wallDistanceL = robot.sonarL.getDistance();
            telemetry.addData("wallDistanceL", wallDistanceL);
            telemetry.addLine("Next: Start");
            if (wallDistanceL > 21 && wallDistanceL < 22) {
                telemetry.addLine("Aligned....Sai!!!!!!!!!!!");
            }
            telemetry.update();
            if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {
                break;
            }
        }
        SelectLinear sl = new SelectLinear(this);
        boolean isRed = sl.selectAlliance();
        boolean isNear = sl.selectPosition();
        boolean isGlyph = sl.extraGlyph();
        //int waitMillis = sl.adjustDelay();


        telemetry.addData("Alliance", isRed ? "Red" : "Blue");
        telemetry.addData("Position", isNear ? "Near" : "Far");
        telemetry.addData("Extra Glyph?", isGlyph ? "Yes" : "No");
        telemetry.addData("Waiting", "AutonomousRevAmped");
        telemetry.update();
        /**
         * if the alliance is red, then show red led. if it is blue, then show blue led
         */
        if (isRed) {
            robot.ledRed.on();
        } else {
            robot.ledBlue.on();
        }
        WaitLinear lp = new WaitLinear(this);
        // lp.waitMillis(waitMillis);

        robot.ledGreen.set(isNear);
        robot.drawLed();
        /**
         * force the enocder of the slide to 0. Reset the heading of the gyro sensor
         * make the container flat so we can put a glyph in at the start of autonomous
         */
        robot.motorSlide.resetPosition();
        robot.gyroSensor.resetHeading();

        waitForStart();

        robot.ledRed.off();
        robot.ledBlue.off();
        robot.ledGreen.off();
        robot.ledWhite.off();
        robot.ledYellow.off();
        robot.drawLed();
        /**
         * initial positions for servos in autonomous
         */
        long startTime = System.currentTimeMillis();
        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
        robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_OUT);
        robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_JEWEL_OUT);
        lp.waitMillis(200);
        /**
         * sense the correct key column on the pictograph using vuforia
         */
        //GlyphDetector distanceBack = new GlyphDetector(this.hardwareMap, "","sensor_back","");
        RelicRecoveryVuMark glyphColumnKey = robot.relicRecoveryVuMark.getGlyphColumnKey();
        int Offset = 0;
        telemetry.addData("GlyphColumnKey:", "%s", glyphColumnKey);
        Boolean isJewelBlue = null;

        if (robot.servoJewel == null) {
            telemetry.addData("jewelservo", "null");
        } else {
            robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_JEWEL_OUT);
        }

        telemetry.update();
        /**
         * use the rev color sensor to detect jewel color and knock jewel off
         * try max 9 times, once see  the color of the jewel then exit.
         */
        for (int i = 1; i < 10; i++) {

            lp.waitMillis(100);

            RevColorDistanceSensor.COLORTYPE jewelcolor = robot.revColorDistanceSensor.getColor();

            if (jewelcolor == RevColorDistanceSensor.COLORTYPE.RED) {
                isJewelBlue = Boolean.FALSE;
                robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_JEWEL_OUT);
                if (isRed) {
                    robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_HIT_LEFT);
                } else {
                    robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_HIT_RIGHT);

                }
                break;
            } else if (jewelcolor == RevColorDistanceSensor.COLORTYPE.BLUE) {
                isJewelBlue = Boolean.TRUE;
                robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_JEWEL_OUT);
                if (isRed) {
                    robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_HIT_RIGHT);
                } else {
                    robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_HIT_LEFT);
                }
                break;
            }
            telemetry.addData("JEWEL", "%s", jewelcolor);
            // send the info back to driver station using telemetry function.

            robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_JEWEL_OUT - i * 3 / 255f);
            telemetry.addData("SERVO", "%5.2f", RobotRevAmpedConstants.SERVO_JEWEL_OUT - i * 3 / 255f);
            telemetry.addData("Alpha", robot.revColorDistanceSensor.getAlpha());
            telemetry.addData("Red  ", robot.revColorDistanceSensor.getRed());
            telemetry.addData("Green", robot.revColorDistanceSensor.getGreen());
            telemetry.addData("Blue ", robot.revColorDistanceSensor.getBlue());
            telemetry.addData("Hue", robot.revColorDistanceSensor.getHue());

            telemetry.update();
        }
        telemetry.update();
        RevColorDistanceSensor.COLORTYPE jewelcolor = robot.revColorDistanceSensor.getColor();
        String strColor;
        if (jewelcolor == RevColorDistanceSensor.COLORTYPE.RED) {
            strColor = "RED";
        } else if (jewelcolor == RevColorDistanceSensor.COLORTYPE.BLUE) {
            strColor = "BLUE";
        } else
            strColor = "NONE";

        telemetry.addData("JEWEL", strColor);
        telemetry.update();
        lp.waitMillis(200);
        robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_JEWEL_IN);
        robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_DETECT);
        if (glyphColumnKey == RelicRecoveryVuMark.RIGHT) {
            drive.moveToEncoderInch(TurnType.FORWARD,
                    -50,
                    0.75f,
                    5000,
                    true,
                    true);
            drive.turn(-104, 0.7f, 2000, false);
        } else if (glyphColumnKey == RelicRecoveryVuMark.CENTER) {
            drive.moveToEncoderInch(TurnType.FORWARD,
                    -40,
                    0.75f,
                    5000,
                    true,
                    true);
            drive.turn(-51, 0.7f, 2000, false);
        } else {
            drive.moveToEncoderInch(TurnType.FORWARD,
                    -48,
                    0.75f,
                    5000,
                    true,
                    true);
            drive.turn(-45, 0.7f, 6000, false);
        }

        lp.waitMillis(100);
        /**
         * dump the glyph and push in
         */
        if (isRed && glyphColumnKey == RelicRecoveryVuMark.RIGHT) {
            drive.moveToTime(-0.3f, 300);
        }
        drive.moveToTime(-0.35f, 500);
        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DUMP);
        lp.waitMillis(800);
        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
        lp.waitMillis(100);
        drive.moveToTime(0.4f, 300);
        drive.moveToTime(-0.4f, 700);
        drive.moveToTime(0.3f, 400);
        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
        /**
         *  if there is enough time, go for the second and third glyph
         */

        long nearTime;
        nearTime = System.currentTimeMillis();
        telemetry.addData("time", nearTime - startTime);
        telemetry.update();
        if (isGlyph && nearTime - startTime < 15000 && (glyphColumnKey == RelicRecoveryVuMark.RIGHT ||
                glyphColumnKey == RelicRecoveryVuMark.LEFT || glyphColumnKey == RelicRecoveryVuMark.CENTER)) {
            robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
            lp.waitMillis(300);
            //correction of heading so that we can go into the glyph pit straight
            if (glyphColumnKey == RelicRecoveryVuMark.RIGHT) {
                drive.turn(28,
                        0.35f,
                        2000,
                        false);
            } else if (glyphColumnKey == RelicRecoveryVuMark.CENTER) {
                drive.turn(-35,
                        0.35f,
                        2000,
                        false);
            } else {
                drive.turn(-30,
                        0.35f,
                        2000,
                        false);
            }
            /**
             *go to the glyph pit
             */
            float sonarCm2;
            sonarCm2 = (robot.sonarRF.getDistance() - 12.7f) * 2 + 72;
            telemetry.addData("sonar distance", sonarCm2);
            telemetry.update();
            robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_OUT);
            robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
            robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
            float startEncoderValue = drive.getEncoder();
            drive.moveToEncoderInch(TurnType.FORWARD,
                    Math.round(sonarCm2 / 2.54f),
                    0.8f,
                    2200,
                    true,
                    true);
            drive.moveToEncoderInch(TurnType.FORWARD,
                    -10,
                    0.75f,
                    2000,
                    true,
                    true);
            int countTray = glyphCount.GetGlyphCount(0);
            if (countTray != 2) {
                /**
                 *  go in twice to get two glyphs. also reverse intake briefly to spit out clogged glyphs
                 */
                robot.sweeperLeft.setPower(-0.6f);
                robot.sweeperRight.setPower(-0.6f);
                lp.waitMillis(400);
                robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                lp.waitMillis(500);
                drive.moveToEncoderInch(TurnType.FORWARD,
                        10,
                        0.7f,
                        600,
                        true,
                        true);
                lp.waitMillis(1000);
                countTray = glyphCount.GetGlyphCount(0);
                if (countTray != 2) {
                    drive.turn(-20,
                            0.4f,
                            2000,
                            false);
                    countTray = glyphCount.GetGlyphCount(0);
                    if (countTray != 2) {
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                20,
                                0.8f,
                                600,
                                true,
                                true);
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                -20,
                                0.8f,
                                600,
                                true,
                                true);
                        lp.waitMillis(100);
                        drive.turn(20,
                                0.4f,
                                2000,
                                false);
                        robot.sweeperLeft.setPower(0);
                        robot.sweeperRight.setPower(0);
                        drive.moveToTime(-0.4f, 800);
                        robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                        lp.waitMillis(600);

                    } else {
                        drive.turn(20,
                                0.5f,
                                2000,
                                false);
                        robot.sweeperLeft.setPower(0);
                        robot.sweeperRight.setPower(0);
                        drive.moveToTime(-0.4f, 800);
                        robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                    }
                } else {
                    robot.sweeperLeft.setPower(0);
                    robot.sweeperRight.setPower(0);
                    drive.moveToTime(-0.4f, 800);
                    robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                }
            } else {
                drive.moveToEncoderInch(TurnType.FORWARD,
                        10,
                        0.75f,
                        2000,
                        true,
                        true);
                robot.sweeperLeft.setPower(0);
                robot.sweeperRight.setPower(0);
                drive.moveToTime(-0.4f, 800);
                robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
            }
            float heading = robot.gyroSensor.getHeading();
            while (heading < -2 || heading > 2) {
                if (heading > 2) {
                    drive.setPower(0.5f, -0.5f);
                } else {
                    drive.setPower(-0.5f, 0.5f);
                }
                heading = robot.gyroSensor.getHeading();
                telemetry.addData("heading", heading);
                telemetry.update();
            }
            /**
             *    go back to the cryptobox
             */
            drive.moveToEncoderInch(TurnType.FORWARD,
                    32 - Math.round(sonarCm2 / 2.54f),
                    0.8f,
                    3000,
                    true,
                    true
            );
            if (glyphColumnKey == RelicRecoveryVuMark.LEFT) {
                drive.moveToEncoderInch(TurnType.STRAFE,
                        -9, 0.5f, 2000, false, true);
                drive.turn(-30,
                        0.4f,
                        2000,
                        false);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
                drive.moveToTime(-0.35f, 600);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                lp.waitMillis(400);
                drive.moveToTime(-0.3f, 300);
                drive.moveToTime(0.3f, 600);
                lp.waitMillis(250);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);

                drive.moveToTime(-0.4f, 700);
                drive.moveToTime(0.3f, 400);

            } else if (glyphColumnKey == RelicRecoveryVuMark.RIGHT) {
                drive.moveToEncoderInch(TurnType.STRAFE,
                        6,
                        0.6f,
                        2000,
                        true,
                        true);
                drive.turn(8,
                        0.5f,
                        2000,
                        false);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
                //robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_OUT);

                drive.moveToTime(-0.4f, 500);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                lp.waitMillis(400);
                drive.moveToTime(0.3f, 600);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);

                drive.moveToTime(-0.35f, 800);
                drive.moveToTime(0.3f, 500);
            } else if (glyphColumnKey == RelicRecoveryVuMark.CENTER) {
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
                drive.moveToEncoderInch(TurnType.STRAFE,
                        -5,
                        0.8f,
                        2000,
                        true,
                        false);

                drive.turn(-14,
                        0.3f,
                        2000,
                        false);

                drive.moveToTime(-0.4f, 500);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                lp.waitMillis(400);
                drive.moveToTime(0.3f, 600);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);

                robot.slide.moveToEncoder(75);
                drive.moveToTime(-0.4f, 800);
                drive.moveToTime(0.3f, 500);
            }
            robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);

        }

    }
}