package org.firstinspires.ftc.teamcode.LastYear.RevAmpedTest;

import android.graphics.Bitmap;

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
import com.qualcomm.robotcore.util.RobotLog;
import com.revAmped.util.JewelDetector;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

/**
 * Autonomous Program for Revamped
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Blue Far Autonomous", group="Game")
public class BlueFarTest
        extends LinearOpMode
{
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
            glyphCount = new GlyphDetector(this.hardwareMap,"", "sensor_back", "");
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
     * @throws InterruptedException
     */
    public void run ()
            throws InterruptedException
    {
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
            if (wallDistanceL>21 && wallDistanceL<22) {
                telemetry.addLine("Aligned....Sai!!!!!!!!!!!");
                robot.ledGreen.on();
            }
            telemetry.update();
            if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {
                break;
            }
        }
        SelectLinear sl = new SelectLinear(this);
        boolean isGlyph = sl.extraGlyph();
        //int waitMillis = sl.adjustDelay();


        telemetry.addData("Extra Glyph?", isGlyph ? "Yes" : "No");
        telemetry.addData("Waiting", "AutonomousRevAmped");
        telemetry.update();
        /**
         * if the alliance is red, then show red led. if it is blue, then show blue led
         */
            robot.ledBlue.on();

        WaitLinear lp = new WaitLinear(this);
        // lp.waitMillis(waitMillis);
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
            robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_INTAKE);
            robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_JEWEL_OUT);
            RelicRecoveryVuMark glyphColumnKey = robot.relicRecoveryVuMark.getGlyphColumnKey();
            lp.waitMillis(300);
            /**
             * sense the correct key column on the pictograph using vuforia
             */
            int leftColumnBlue = 19;
            int centerColumnBlue = 21;
            int rightColumnBlue = 24;
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
            try {
                Bitmap bitmap = robot.relicRecoveryVuMark.vuforia.getBitmap();
                Mat origimg = new Mat(bitmap.getHeight(), bitmap.getWidth(), CvType.CV_8UC4);   // 8 bits  3 channel
                Utils.bitmapToMat(bitmap, origimg);

                JewelDetector jd = new JewelDetector();
                //RevColorDistanceSensor.COLORTYPE jewel = jd.GetJewelColor(origimg);
                RevColorDistanceSensor.COLORTYPE jewel = jd.GetJewelColor(origimg);
                telemetry.addData("REVAMPED_JEWEL", "%s", jewel);
                RobotLog.vv("OPCV","%s", jewel);
                //jd.processFrame(origimg, true);
                //JewelDetector.JewelOrder order = jd.getCurrentOrder();
                //telemetry.addData("DOGCV_JEWEL", "%s", order);
                //telemetry.addData("Jewel", "%s", jewel);
                //telemetry.addData("DOGECV", "%s", order);
                telemetry.update();
                // for fun,  detect the hexgons.
                // jd.getHexgonPattern(origimg);
                if (jewel == RevColorDistanceSensor.COLORTYPE.BLUE&&glyphColumnKey!=RelicRecoveryVuMark.UNKNOWN) {
                    robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_HIT_RIGHT);
                } else if (jewel == RevColorDistanceSensor.COLORTYPE.RED&&glyphColumnKey!=RelicRecoveryVuMark.UNKNOWN) {
                    robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_HIT_LEFT);
                } else {
                    for (int i = 1; i < 6; i++) {

                        lp.waitMillis(50);

                        RevColorDistanceSensor.COLORTYPE jewelcolor = robot.revColorDistanceSensor.getColor();

                        if (jewelcolor == RevColorDistanceSensor.COLORTYPE.RED) {

                            robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_HIT_RIGHT);
                            break;
                        } else if (jewelcolor == RevColorDistanceSensor.COLORTYPE.BLUE) {

                            robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_HIT_LEFT);
                            break;
                        }
                        telemetry.addData("JEWEL", "%s", jewelcolor);
                        // send the info back to driver station using telemetry function.

                        robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_JEWEL_OUT - i * 2.5f / 255f);
                        robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_DETECT - i * 2 / 255f);
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

        /*for (int i = 1; i < 8; i++) {

            lp.waitMillis(50);

            RevColorDistanceSensor.COLORTYPE jewelcolor = robot.revColorDistanceSensor.getColor();

            if (jewelcolor == RevColorDistanceSensor.COLORTYPE.RED )
            {
                    robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_HIT_RIGHT);
                    break;
            }
            else if (jewelcolor == RevColorDistanceSensor.COLORTYPE.BLUE)
            {
                    robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_HIT_LEFT);
                    break;

            }
            telemetry.addData("JEWEL", "%s", jewelcolor);
            // send the info back to driver station using telemetry function.

            robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_JEWEL_OUT - i*2.5f/255f);
            robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_DETECT-i*2/255f);
            telemetry.addData("SERVO", "%5.2f", RobotRevAmpedConstants.SERVO_JEWEL_OUT - i*3/255f);
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

        telemetry.addData("JEWEL", strColor);*/
            telemetry.update();
            lp.waitMillis(300);
            robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_JEWEL_IN);
            robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_DETECT);

            drive.setPower(0.56f, -0.5f, -0.4f, 0.6f);
            lp.waitMillis(1500);
            float startHeading = robot.gyroSensor.getHeading();
            while (startHeading < -1.5 || startHeading > 1.5) {
                if (startHeading < -1.5) {
                    drive.setPower(-0.4f, 0.4f);
                } else if (startHeading > 1.5) {
                    drive.setPower(0.4f, -0.4f);
                }
                startHeading = robot.gyroSensor.getHeading();
                telemetry.addData("heading", startHeading);
                telemetry.update();
            }
            drive.turn(-142,
                    0.8f,
                    4000,
                    true);
            robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
            robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
            Boolean FirstGlyph = false;
            drive.moveToTime(0.5f, 500);
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
                while (countTray != 2 && endTime < (startTimestamp + 600)) {
                    robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                    robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                    lp.waitMillis(100);
                    countTray = glyphCount.GetGlyphCount(0);
                    endTime = System.currentTimeMillis();

                }
            }
            //lp.waitMillis(100);
            robot.sweeperRight.setPower(0.6f);
            robot.sweeperLeft.setPower(-0.6f);
            if (FirstGlyph) {
                drive.moveToTime(-0.5f, 600);
            } else {
                drive.moveToTime(-0.5f, 400);

            }
            robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
            robot.sweeperRight.setPower(0.6f);
            robot.sweeperLeft.setPower(0.6f);
            drive.turn(-30,
                    0.4f,
                    2000,
                    false);
            drive.moveToEncoderInch(TurnType.FORWARD,
                    -22,
                    0.8f,
                    3000,
                    true,
                    true);
            Boolean isJam = robot.switchDoor.isTouch();
            /*if (!isJam) {
                robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_INTAKE);
                robot.sweeperRight.setPower(-0.6f);
                robot.sweeperLeft.setPower(-0.6f);
                lp.waitMillis(500);
                robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
            }*/
            robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_IN);
            robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
            robot.sweeperRight.setPower(0);
            robot.sweeperLeft.setPower(0);
            float sonarWallBlue = robot.sonarR.getDistance();

            if (glyphColumnKey == RelicRecoveryVuMark.LEFT) {
                if (sonarWallBlue < 20.4) {
                    telemetry.addLine("hi");
                    telemetry.update();
                    drive.moveToEncoderInch(TurnType.STRAFE,
                            5,
                            0.8f,
                            8000,
                            true,
                            false);

                }
                sonarWallBlue = robot.sonarR.getDistance();
                while (sonarWallBlue >20.4) {
                    drive.setPower(0.6f, -0.6f, -0.6f, 0.6f);
                    telemetry.addData("distance from wall", sonarWallBlue);
                    telemetry.update();
                    sonarWallBlue = robot.sonarR.getDistance();
                }
                drive.turn(18,
                        0.5f,
                        4000,
                        false);
            } else if (glyphColumnKey == RelicRecoveryVuMark.CENTER) {
                //drive.moveToTime(-0.35f, 400);
                if (sonarWallBlue < /*22.5*/22.4) {
                    telemetry.addLine("hi");
                    drive.moveToEncoderInch(TurnType.STRAFE,
                            6,
                            0.3f,
                            8000,
                            true,
                            true);
                    telemetry.update();

                }
                sonarWallBlue = robot.sonarR.getDistance();
                while (sonarWallBlue >/*22.5*/22.4) {
                    drive.setPower(0.6f, -0.6f, -0.6f, 0.6f);
                    telemetry.addData("distance from wall", sonarWallBlue);
                    telemetry.update();
                    sonarWallBlue = robot.sonarR.getDistance();
                }
                /*drive.turn(16,
                        0.45f,
                        4000,
                        false);*/
                drive.setTurnPower(0.4f);
                lp.waitMillis(432);
            } else {
                drive.moveToTime(0.4f, 300);
                if (sonarWallBlue < 24 && sonarWallBlue > 23) {
                    drive.moveToTime(-0.3f, 300);

                } else if (sonarWallBlue < 23) {
                    drive.moveToEncoderInch(TurnType.STRAFE,
                            -2,
                            0.5f,
                            3000,
                            false,
                            true);
                }
                sonarWallBlue = robot.sonarR.getDistance();
                while (sonarWallBlue > 24.4) {
                    drive.setPower(0.6f, -0.6f, -0.6f, 0.6f);
                    telemetry.addData("distance from wall", sonarWallBlue);
                    sonarWallBlue = robot.sonarR.getDistance();
                }
                /*drive.turn(13,
                            0.4f,
                            4000,
                            false);*/
                drive.setTurnPower(0.4f);
                lp.waitMillis(420);

            }
            /**
             *  dump the first glyph
             */


            robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
            drive.moveToTime(-0.45f, 400);
            lp.waitMillis(200);
            countTray = glyphCount.GetGlyphCount(0);
            if (countTray == 2) {
                robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_INTAKE);
                lp.waitMillis(300);
                robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                lp.waitMillis(100);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                lp.waitMillis(400);
            }
            robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
            drive.moveToTime(-0.55f, 750);
            lp.waitMillis(100);
            drive.moveToTime(0.5f, 350);
            robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
            /**
             *   for second and third glyphs
             */
            long farTime;
            farTime = System.currentTimeMillis();
            if (isGlyph && farTime - startTime < 18000 /*&& (glyphColumnKey==RelicRecoveryVuMark.RIGHT ||
                    glyphColumnKey==RelicRecoveryVuMark.LEFT || glyphColumnKey==RelicRecoveryVuMark.CENTER)*/) {


                if ((glyphColumnKey == RelicRecoveryVuMark.CENTER)/* || (glyphColumnKey==RelicRecoveryVuMark.LEFT)*/) {
                    drive.turn(-7,
                            0.5f,
                            5000,
                            false
                    );
                    drive.moveToEncoderInch(TurnType.STRAFE,
                            -3, 0.6f, 3000, false, true);

                } else if (glyphColumnKey == RelicRecoveryVuMark.LEFT) {
                  /*drive.turn(-7,
                          0.5f,
                          5000,
                          false
                  );*/
                    drive.setTurnPower(-0.4f);
                    lp.waitMillis(200);
                  /*drive.moveToEncoderInch(TurnType.FORWARD,
                          5,
                          0.6f,
                          3000,
                          true,
                          true);*/
                    drive.moveToEncoderInch(TurnType.STRAFE,
                            -7,
                            0.75f,
                            3000,
                            true, true);

                  /*drive.setPower(-0.344f, 0.49f, 0.49f, -0.344f);
                  lp.waitMillis(600);*/
                } else {
                    drive.moveToEncoderInch(TurnType.STRAFE, 7, 0.6f, 4000,
                            false,
                            true);

                }

                /*float sonarInch = (((robot.sonarRF.getDistance()-12.7f)*2) + 65)/2;*/
                Boolean MoreGlyphs = false;
                robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_INTAKE + 50);
                robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                drive.moveToEncoderInch(TurnType.FORWARD,
                        44,
                        0.85f,
                        2200,
                        true,
                        true);
                lp.waitMillis(300);
                int countTrayFar = glyphCount.GetGlyphCount(0);
                int i2 = 0;
                if (countTrayFar != 2) {
                    long timePit2 = System.currentTimeMillis();
                    long endTimePit2 = timePit2;
                    while (countTrayFar != 2 && endTimePit2 < (timePit2 + 3500)) {
                        if (i2 % 2 == 0) {
                            drive.setTurnPower(0.4f);
                            lp.waitMillis(250 + 20 * i2);
                            drive.moveToTime(0.45f, 500 + 40 * i2);
                        } else {
                            drive.setTurnPower(-0.4f);
                            lp.waitMillis(250 + 20 * i2);
                            drive.moveToTime(0.45f, 500 + 40 * i2);
                        }
                        drive.moveToTime(-0.45f, 465 + 35 * i2);
                        endTimePit2 = System.currentTimeMillis();
                        lp.waitMillis(50);
                        countTrayFar = glyphCount.GetGlyphCount(0);
                        i2++;
                    }
                    RobotLog.vv("number of tries", "%d", i2);
                    robot.sweeperRight.setPower(0.6f);
                    robot.sweeperLeft.setPower(-0.6f);
                    if (glyphColumnKey == RelicRecoveryVuMark.LEFT || glyphColumnKey==RelicRecoveryVuMark.CENTER) {
                        drive.moveToTime(-0.5f, 500);
                    }
                    MoreGlyphs = true;
                    robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                } else {
                    drive.moveToEncoderInch(TurnType.FORWARD,
                            -8,
                            0.5f,
                            3000,
                            false,
                            true);
                    drive.turn(-14, 0.4f, 3000, false);
                    MoreGlyphs = false;
                    robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                }

                robot.sweeperLeft.setPower(0);
                robot.sweeperRight.setPower(0);
                if (/*glyphColumnKey==RelicRecoveryVuMark.CENTER ||
                        glyphColumnKey ==RelicRecoveryVuMark.LEFT &&*/ MoreGlyphs && i2 != 0) {
                    float heading = robot.gyroSensor.getHeading();
                    while (heading < -1 || heading > 1) {
                        if (heading > 1) {
                            drive.setPower(0.5f, -0.5f);
                        } else {
                            drive.setPower(-0.5f, 0.5f);
                        }
                        heading = robot.gyroSensor.getHeading();
                        telemetry.addData("heading", heading);
                        telemetry.update();
                    }
                    drive.setTurnPower(0.4f);
                    lp.waitMillis(200);
                    if (glyphColumnKey == RelicRecoveryVuMark.LEFT) {
                        drive.setTurnPower(-0.4f);
                        lp.waitMillis(200);
                    }
                    /*if (i2%2!=0) {
                        drive.setTurnPower(-0.4f);
                        lp.waitMillis(250);
                    }
                    drive.turn(-15,
                            0.5f,
                            5000,
                            false);*/
                }
                robot.sweeperRight.setPower(0.6f);
                robot.sweeperLeft.setPower(0.6f);
                /*isJam = robot.switchDoor.isTouch();
                if (!isJam) {
                    robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_INTAKE);
                    robot.sweeperRight.setPower(-0.6f);
                    robot.sweeperLeft.setPower(-0.6f);
                    lp.waitMillis(500);
                    robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                }*/
                drive.moveToEncoderInch(TurnType.FORWARD,
                        -32,
                        0.85f,
                        1800,
                        true,
                        true);
                robot.sweeperRight.setPower(0);
                robot.sweeperLeft.setPower(0);
                lp.waitMillis(100);
                float sonarWallBlue2 = robot.sonarR.getDistance();
                if (glyphColumnKey == RelicRecoveryVuMark.LEFT) {
                    /*while (sonarWallBlue2>23) {
                        drive.setPower(0.5f, -0.5f, -0.5f, 0.5f);
                        telemetry.addData("distance from wall", sonarWallBlue2);
                        telemetry.update();
                        sonarWallBlue2 = robot.sonarR.getDistance();
                    }*/

                        /*drive.moveToEncoderInch(TurnType.STRAFE,
                                12,
                                0.6f,
                                3000,
                                true,
                                true);*/

                    drive.moveToTime(0.3f, 400);
                    drive.moveToEncoderInch(TurnType.STRAFE,
                            6,
                            0.5f,
                            3000,
                            false,
                            false);

                    robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_UP);
                    /*drive.setPower(-0.4f, 0.4f);
                    lp.waitMillis(350);*/
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                    //drive.moveToTime(-0.4f, 400);
                    drive.moveToTime(-0.35f, 450);
                    robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);

                    lp.waitMillis(100);
                    drive.moveToTime(-0.45f, 550);
                    drive.moveToTime(0.5f, 500);
                    drive.setStrafePower(0.5f);
                    lp.waitMillis(350);
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);

                } else if (glyphColumnKey == RelicRecoveryVuMark.CENTER) {
                    if (sonarWallBlue2 < 23.4) {
                        if (sonarWallBlue < 23.4 && sonarWallBlue > 20.4) {
                        } else {
                            while (sonarWallBlue2 < 20.4) {
                                drive.setPower(-0.4f, 0.4f, 0.4f, -0.4f);
                                telemetry.addData("distance from wall", sonarWallBlue2);
                                telemetry.update();
                                sonarWallBlue2 = robot.sonarR.getDistance();
                            }
                        }
                    } else {
                        while (sonarWallBlue2 > 23.4) {
                            drive.setPower(0.5f, -0.5f, -0.5f, 0.5f);
                            telemetry.addData("distance from wall", sonarWallBlue2);
                            telemetry.update();
                            sonarWallBlue2 = robot.sonarR.getDistance();
                        }
                    }
                    robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_UP);
                    /*drive.turn(15,
                            0.5f,
                            3000,
                            false
                    );*/
                    drive.moveToTime(-0.4f, 300);
                    drive.setPower(-0.4f, 0.4f);
                    lp.waitMillis(360);
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                    drive.moveToTime(-0.48f, 600);
                    robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
                    drive.moveToTime(-0.3f, 450);
                    lp.waitMillis(100);
                    drive.moveToTime(0.5f, 500);
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                } else {


                    robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_IN);
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
                    /*if (sonarWallBlue2>18.5) {
                     } else {*/
                    drive.moveToEncoderInch(TurnType.STRAFE,
                            20,
                            0.8f,
                            5000,
                            true,
                            true);
                    sonarWallBlue2 = robot.sonarR.getDistance();
                    while (sonarWallBlue2 > 20.4) {
                        drive.setPower(0.5f, -0.5f, -0.5f, 0.5f);
                        telemetry.addData("distance from wall", sonarWallBlue2);
                        telemetry.update();
                        sonarWallBlue2 = robot.sonarR.getDistance();
                    }
                    robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_UP);
                    /*drive.turn(14,
                            0.5f,
                            3000,
                            false
                    );*/
                    drive.setPower(-0.4f, 0.4f);
                    lp.waitMillis(360);
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                    drive.moveToTime(-0.45f, 500);
                    robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
                    drive.moveToTime(-0.5f, 340);
                    lp.waitMillis(100);
                    drive.moveToTime(0.5f, 500);
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                }
                robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);

            }
            long timeStamp = System.currentTimeMillis();
            drive.stop();

            RobotLog.i("AutonomousRevamped finish in " + (System.currentTimeMillis() - startTime) + " milliseconds");
            lp.waitMillis(100);
        } catch (Exception e) {drive.stop();}
    }
}



