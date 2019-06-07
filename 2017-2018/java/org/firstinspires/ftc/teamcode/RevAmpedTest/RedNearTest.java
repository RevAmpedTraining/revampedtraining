package org.firstinspires.ftc.teamcode.LastYear.RevAmpedTest;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.util.RobotLog;
import com.revAmped.components.Button;
import com.revAmped.components.HwSonarAnalog;
import com.revAmped.components.RobotRevAmped;
import com.revAmped.components.TurnType;
import com.revAmped.config.RobotRevAmpedConstants;
import com.revAmped.linear.components.MecanumDriveLinear;
import com.revAmped.linear.components.RobotRevAmpedLinear2;
import com.revAmped.linear.util.SelectLinear;
import com.revAmped.linear.util.WaitLinear;
import com.revAmped.sensors.RevColorDistanceSensor;
import com.revAmped.util.GlyphDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.revAmped.util.JewelDetector;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

/**
 * Test autonomous for Blue Near
 * RevAmped 12808
 * Created Pi Day 2018
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Red Near Autonomous", group="Game")
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
     * align the glyphs using range sensor
     * @throws InterruptedException
     */
    public void alignToCryptobox(WaitLinear lp, double low, double high)
            throws InterruptedException {
        double alignDistance = robot.rangeStick.getDistance(DistanceUnit.CM);
        robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_IN);
        long timeStamp = System.currentTimeMillis();
        long endTime = timeStamp;
        while (alignDistance < low || alignDistance > high && endTime<(timeStamp+2300)) {
            if (alignDistance > high) {
                drive.setStrafePower(0.32f);
                lp.waitMillis(150);
            } else/* if (alignDistance < low)*/ {
                drive.setStrafePower(-0.32f);
                lp.waitMillis(150);
            }
            drive.stop();
            endTime=System.currentTimeMillis();
            alignDistance = robot.rangeStick.getDistance(DistanceUnit.CM);
        }
        drive.moveToTime(0.5f, 200);
        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
        lp.waitMillis(500);
        drive.moveToTime(-0.5f, 350);
        robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
        drive.moveToTime(0.4f, 400);
        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
        robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_UP);
    }
    /**
     * dump the glyphs
     * @throws InterruptedException
     */
    public void dump ()
            throws InterruptedException {
        drive.moveToTime(-0.5f, 600);
        robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
        Thread.sleep(150);
        drive.moveToTime(0.5f, 300);
        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
        robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_IN);

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
        boolean fourGlyph = sl.extraGlyph();
        boolean sixGlyph = false;
        if (fourGlyph) {
            sixGlyph = sl.extraGlyph2();
        }

        //int waitMillis = sl.adjustDelay();


        // telemetry.addData("Alliance", isRed ? "Red" : "Blue");
        //telemetry.addData("Position", isNear ? "Near" : "Far");
        telemetry.addData("4 Glyph?", fourGlyph ? "Yes" : "No");
        telemetry.addData("6 glyph?", sixGlyph? "Yes" : "No");
        telemetry.addData("Waiting", "AutonomousRevAmped");
        telemetry.update();
        /**
         * if the alliance is red, then show red led. if it is blue, then show blue led
         */
        robot.ledBlue.on();

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
        try {
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
            //robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
            robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_INTAKE + 15);
            robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_JEWEL_OUT);
            lp.waitMillis(300);
            /**
             * sense the correct key column on the pictograph using vuforia
             */
            //GlyphDetector distanceBack = new GlyphDetector(this.hardwareMap, "","sensor_back","");
            RelicRecoveryVuMark glyphColumnKey = robot.relicRecoveryVuMark.getGlyphColumnKey();
            int leftOffset = -17;
            int centerOffset = -8;
            int rightOffset = -1;
            int Offset = rightOffset;
            telemetry.addData("GlyphColumnKey:", "%s", glyphColumnKey);
            if (glyphColumnKey == RelicRecoveryVuMark.LEFT) {
                Offset = leftOffset;
            } else if (glyphColumnKey == RelicRecoveryVuMark.CENTER) {
                Offset = centerOffset;
            } else {
                Offset = rightOffset;
            }

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
            try {
                Bitmap bitmap = robot.relicRecoveryVuMark.vuforia.getBitmap();
                Mat origimg = new Mat(bitmap.getHeight(), bitmap.getWidth(), CvType.CV_8UC4);   // 8 bits  3 channel
                Utils.bitmapToMat(bitmap, origimg);

                JewelDetector jd = new JewelDetector();
                //RevColorDistanceSensor.COLORTYPE jewel = jd.GetJewelColor(origimg);
                RevColorDistanceSensor.COLORTYPE jewel = jd.GetJewelColor(origimg);
                telemetry.addData("REVAMPED_JEWEL", "%s", jewel);
                RobotLog.vv("OPENCV",  "%s", jewel);
                //jd.processFrame(origimg, true);
                //JewelDetector.JewelOrder order = jd.getCurrentOrder();
                //telemetry.addData("DOGCV_JEWEL", "%s", order);
                //telemetry.addData("Jewel", "%s", jewel);
                //telemetry.addData("DOGECV", "%s", order);
                telemetry.update();
                // for fun,  detect the hexgons.
                // jd.getHexgonPattern(origimg);
                if (jewel == RevColorDistanceSensor.COLORTYPE.BLUE&&glyphColumnKey!=RelicRecoveryVuMark.UNKNOWN) {
                    robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_HIT_LEFT);
                } else if (jewel == RevColorDistanceSensor.COLORTYPE.RED&&glyphColumnKey!=RelicRecoveryVuMark.UNKNOWN) {
                    robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_HIT_RIGHT);
                } else if (jewel== RevColorDistanceSensor.COLORTYPE.NONE) {
                    Boolean isJewelBlue = null;
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
                        RobotLog.vv("COLORSENSOR",  "%s", jewelcolor);
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
                }
                //origimg.release();
                robot.relicRecoveryVuMark.stop();
            } catch (Exception e) {
                RobotLog.e("Vuforia " + e.getMessage());
            }

        /*for (int i = 1; i < 10; i++) {

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
        }*/
            telemetry.update();
        /*RevColorDistanceSensor.COLORTYPE jewelcolor = robot.revColorDistanceSensor.getColor();
        String strColor;
        if (jewelcolor == RevColorDistanceSensor.COLORTYPE.RED) {
            strColor = "RED";
        } else if (jewelcolor == RevColorDistanceSensor.COLORTYPE.BLUE) {
            strColor = "BLUE";
        } else
            strColor = "NONE";

        telemetry.addData("JEWEL", strColor);
        telemetry.update();*/
            lp.waitMillis(300);
            robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_JEWEL_IN);
            robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_DETECT);
            robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
            robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
            drive.moveToEncoderInch(TurnType.FORWARD,
                    -29 + Offset,
                    0.8f,
                    4000,
                    true,
                    false);
            float headingStablize1 = robot.gyroSensor.getHeading();
            while (headingStablize1 > -72) {
                drive.setPower(0.4f, 0.4f, -0.4f, 0);
                headingStablize1 = robot.gyroSensor.getHeading();
                RobotLog.aa("heading turn 1", "%f", headingStablize1);
            }
            drive.turn(12,
                    0.6f,
                    4000,
                    false);
            Boolean FirstGlyph = false;
            drive.moveToTime(0.5f, 800);
            lp.waitMillis(400);
            int countTray = glyphCount.GetGlyphCount(0);
            if (countTray != 2) {
                FirstGlyph = true;
                long timestamp = System.currentTimeMillis();
                drive.moveToTime(0.5f, 300);
                lp.waitMillis(100);
                countTray = glyphCount.GetGlyphCount(0);
                long startTimestamp = System.currentTimeMillis();
                long endTime = startTimestamp;
                while (countTray != 2 && endTime < (startTimestamp + 400)) {
                    robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                    robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                    countTray = glyphCount.GetGlyphCount(0);
                    endTime = System.currentTimeMillis();

                }
            }
            robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
            robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_UP);
            //lp.waitMillis(100);
            robot.sweeperRight.setPower(0.6f);
            robot.sweeperLeft.setPower(-0.6f);
            if (FirstGlyph) {
                drive.moveToTime(-0.5f, 1200);
            } else {
                drive.moveToTime(-0.5f, 900);

            }
            robot.sweeperRight.setPower(0.6f);
            robot.sweeperLeft.setPower(0.6f);
            drive.moveToTime(-0.5f, 600);
            robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_OUT);
            if (glyphColumnKey==RelicRecoveryVuMark.RIGHT) {
                robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_OUT-3f/255f);
                drive.moveToEncoderInch(TurnType.FORWARD, 4, 0.4f,
                        3000,
                        false,
                        true);
            }
            else {
                drive.moveToEncoderInch(TurnType.FORWARD, 3, 0.4f,
                        3000,
                        false,
                        true);
            }
            //drive.moveToTime(0.5f, 200);

        /*double distanceBack = robot.rangeStick.getDistance(DistanceUnit.CM);
        while (distanceBack>25) {
            drive.setPower(-0.3f);
            distanceBack = robot.rangeStick.getDistance(DistanceUnit.CM);
        }*/
            //drive.moveToTime(0.4f, 160);
            robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_IN);
            robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLATA);
            lp.waitMillis(100);
            alignToCryptobox(lp, 6.25, 6.75);
            drive.setTurnPower(0.4f);
            lp.waitMillis(60);
            if (fourGlyph) {
                if (glyphColumnKey == RelicRecoveryVuMark.LEFT) {
                    drive.moveToEncoderInch(TurnType.STRAFE,
                            centerOffset-5,
                            0.6f,
                            3000,
                            false,
                            true);
                } else if (glyphColumnKey == RelicRecoveryVuMark.CENTER) {
                    drive.moveToEncoderInch(TurnType.STRAFE,
                            centerOffset-1,
                            0.6f,
                            3000,
                            false,
                            true);
                } else {
                    drive.moveToEncoderInch(TurnType.STRAFE,
                            -centerOffset + 1,
                            0.6f,
                            3000,
                            false,
                            true);
                }
                robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_INTAKE + 8);
                robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                float sonarCm2;
                sonarCm2 = (robot.sonarRF.getDistance() - 12.7f) * 2 + 50;
                if (sonarCm2 > 90) {
                    sonarCm2 = 90;
                }
                telemetry.addData("sonar distance", sonarCm2);
                telemetry.update();
                drive.moveToEncoderInch(TurnType.FORWARD,
                        Math.round(sonarCm2 / 2.54f),
                        0.8f,
                        2400,
                        true,
                        true);
                drive.moveToTime(-0.4f, 300);
                countTray = glyphCount.GetGlyphCount(0);
                int i2 = 0;
                if (countTray != 2) {
                    for (int i = 0; i < 3; i++) {
                        i2++;
                        robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                        robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                        if (i == 0) {
                            drive.moveToTime(0.5f, 650);
                            lp.waitMillis(100);
                        } else if (i == 1) {
                        /*drive.setTurnPower(0.4f);
                        lp.waitMillis(250);*/
                            drive.moveToTime(0.45f, 600);
                            lp.waitMillis(100);

                        } else if (i == 2) {
                        /*drive.setTurnPower(-0.4f);
                        lp.waitMillis(500);*/
                            drive.moveToTime(0.45f, 600);
                            lp.waitMillis(100);

                        }
                        drive.moveToTime(-0.45f, 500);
                        robot.sweeperRight.setPower(0.6f);
                        robot.sweeperLeft.setPower(-0.6f);
                        lp.waitMillis(50);
                        countTray = glyphCount.GetGlyphCount(0);
                        if (countTray == 2) {
                            break;
                        }
                        i++;


                    }
                    robot.sweeperRight.setPower(0.6f);
                    robot.sweeperLeft.setPower(-0.6f);
                /*if (i2==2) {
                    drive.setPower(-0.4f, 0.4f);
                    lp.waitMillis(500);
                }else if (i2==3) {
                    drive.setPower(0.4f, -0.4f);
                    lp.waitMillis(500);
                }*/
                    drive.moveToEncoderInch(TurnType.FORWARD,
                            -5,
                            0.6f,
                            3000,
                            false,
                            true);
                    float headingStablize = robot.gyroSensor.getHeading();
                    while (headingStablize > -103 || headingStablize < -105) {
                        if (headingStablize > -103) {
                            drive.setPower(0.4f, 0.4f, -0.4f, -0.4f);
                        } else {
                            drive.setTurnPower(0.4f);
                        }
                        headingStablize = robot.gyroSensor.getHeading();
                        RobotLog.aa("heading turn 1", "%f", headingStablize);
                    }
                    if (glyphColumnKey==RelicRecoveryVuMark.CENTER || glyphColumnKey==RelicRecoveryVuMark.LEFT) {
                        drive.setTurnPower(-0.4f);
                        lp.waitMillis(80);
                    }
                    robot.sweeperLeft.setPower(0);
                    robot.sweeperRight.setPower(0);
                    robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                    robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_UP);
                    // robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_OUT);
                    drive.moveToEncoderInch(TurnType.FORWARD,
                            15 - Math.round(sonarCm2 / 2.54f),
                            0.8f,
                            2000,
                            true,
                            true);
                    //drive.moveToTime(-0.55f, 500);
                    robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_OUT_2);
                    long time = System.currentTimeMillis();
                    long endT = time;
                    double distanceBack = robot.rangeStick.getDistance(DistanceUnit.CM);
                    while (distanceBack > 20 && endT < (time+4000)) {
                        drive.setPower(-0.29f);
                        distanceBack = robot.rangeStick.getDistance(DistanceUnit.CM);
                        endT = System.currentTimeMillis();
                    }
                    drive.moveToTime(0.4f, 160);
                } else {
                    drive.moveToEncoderInch(TurnType.FORWARD,
                            -10,
                            0.6f,
                            3000,
                            false,
                            true);
                    robot.sweeperLeft.setPower(0);
                    robot.sweeperRight.setPower(0);
                /*float heading = robot.gyroSensor.getHeading();
                while (heading < -1 || heading > 1) {
                    if (heading > 1) {
                        drive.setPower(0.5f, -0.5f);
                    } else {
                        drive.setPower(-0.5f,
                    heading = robot.gyroSensor.ge0.5f);
                    }tHeading();
                    telemetry.addData("heading", heading);
                    telemetry.update();
                }*/
                    float headingStablize = robot.gyroSensor.getHeading();
                    while (headingStablize > -104 || headingStablize < -106) {
                        if (headingStablize > -104) {
                            drive.setPower(0.4f, 0.4f, -0.4f, -0.4f);
                        } else {
                            drive.setTurnPower(0.4f);
                        }
                        headingStablize = robot.gyroSensor.getHeading();
                        RobotLog.aa("heading turn 1", "%f", headingStablize);
                    }
                    if (glyphColumnKey==RelicRecoveryVuMark.CENTER || glyphColumnKey==RelicRecoveryVuMark.LEFT) {
                        drive.setTurnPower(-0.4f);
                        lp.waitMillis(60);
                    }
                    robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                    robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_UP);

                    drive.moveToEncoderInch(TurnType.FORWARD,
                            15 - Math.round(sonarCm2 / 2.54f),
                            0.8f,
                            2200,
                            true,
                            true);


                /*if (glyphColumnKey!=RelicRecoveryVuMark.CENTER) {
                    drive.moveToTime(-0.4f, 500);
                }*/

               /*double distanceBack = robot.rangeStick.getDistance(DistanceUnit.CM);
                while (distanceBack>20) {
                    drive.setPower(-0.32f);
                    distanceBack = robot.rangeStick.getDistance(DistanceUnit.CM);
                }*/
                    long time = System.currentTimeMillis();
                    long endT = time;
                    double distanceBack = robot.rangeStick.getDistance(DistanceUnit.CM);
                    while (distanceBack > 20 && endT < (time+4000)) {
                        drive.setPower(-0.29f);
                        distanceBack = robot.rangeStick.getDistance(DistanceUnit.CM);
                        endT = System.currentTimeMillis();
                    }
                    drive.moveToTime(0.4f, 160);
                /*drive.moveToTime(-0.5f, 500);
                robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_OUT);
                drive.moveToEncoderInch(TurnType.FORWARD,
                        1,
                        0.4f,
                        2000,
                        false,
                        true);*/
                }
                robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_IN);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLATA);
                lp.waitMillis(50);
                //if (glyphColumnKey!=RelicRecoveryVuMark.CENTER) {
                alignToCryptobox(lp, 6.6, 7.1);
                //} else {

            }
            long nearTime = System.currentTimeMillis();
            if (sixGlyph && (startTime - nearTime) < 20000) {
                try {
                    float headingStablize = robot.gyroSensor.getHeading();
                    while (headingStablize > -103 || headingStablize < -105) {
                        if (headingStablize > -103) {
                            drive.setPower(0.4f, 0.4f, -0.4f, -0.4f);
                        } else {
                            drive.setTurnPower(0.4f);
                        }
                        headingStablize = robot.gyroSensor.getHeading();
                        RobotLog.aa("heading turn 1", "%f", headingStablize);
                    }
                    if (glyphColumnKey == RelicRecoveryVuMark.CENTER) {
                        drive.moveToEncoderInch(TurnType.STRAFE,
                                -centerOffset - 2,
                                0.6f,
                                4000,
                                false,
                                true);
                    }
                    robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_INTAKE + 15);
                    robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                    robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                    float sonarCm2 = (robot.sonarRF.getDistance() - 12.7f) * 2 + 50;
                    if (sonarCm2 > 100) {
                        sonarCm2 = 100;
                    }
                    telemetry.addData("sonar distance", sonarCm2);
                    telemetry.update();
                    drive.moveToEncoderInch(TurnType.FORWARD,
                            Math.round(sonarCm2 / 2.54f),
                            0.85f,
                            2400,
                            false,
                            true);
                    lp.waitMillis(200);
                    drive.moveToTime(-0.5f, 200);
                    countTray = glyphCount.GetGlyphCount(0);
                    long timeLastTrip = System.currentTimeMillis();
                    /*if (countTray != 2 && (timeLastTrip - startTime) < 25000) {
                        drive.setTurnPower(0.4f);
                        lp.waitMillis(275);
                        drive.moveToTime(0.5f, 450);
                        lp.waitMillis(200);
                        drive.moveToTime(-0.5f, 450);
                        drive.setTurnPower(-0.4f);
                        lp.waitMillis(275);
                    }*/
                    headingStablize = robot.gyroSensor.getHeading();
                    while (headingStablize > -103 || headingStablize < -105) {
                        if (headingStablize > -103) {
                            drive.setPower(0.4f, 0.4f, -0.4f, -0.4f);
                        } else {
                            drive.setTurnPower(0.4f);
                        }
                        headingStablize = robot.gyroSensor.getHeading();
                        RobotLog.aa("heading turn 1", "%f", headingStablize);
                    }
                    drive.setTurnPower(-0.4f);
                    lp.waitMillis(225);
                    robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                    robot.sweeperLeft.setPower(0);
                    robot.sweeperRight.setPower(0);

                    drive.moveToEncoderInch(TurnType.FORWARD,
                            6 - Math.round(sonarCm2 / 2.54f),
                            0.8f,
                            2200,
                            false,
                            true);
                    robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_IN);
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                    if (glyphColumnKey == RelicRecoveryVuMark.LEFT) {
                        //drive.turn(25, 0.6f, 4000, false);
                        drive.setTurnPower(-0.42f);
                        lp.waitMillis(270);
                    } else {
                        //drive.turn(-25, 0.6f, 4000, false);
                        drive.setTurnPower(0.42f);
                        lp.waitMillis(270);
                    }
                    dump();
                    //}
                    robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                    robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
                    drive.stop();
                } catch (Exception e) {
                    drive.stop();
                }
                long timeStamp = System.currentTimeMillis();
                drive.stop();

                RobotLog.i("AutonomousRevamped finish in " + (System.currentTimeMillis() - startTime) + " milliseconds");
                lp.waitMillis(50);
            }
        } catch (Exception e) {drive.stop();}

    }

}