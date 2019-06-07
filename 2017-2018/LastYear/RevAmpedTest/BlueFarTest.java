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
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * Autonomous Program for Revamped
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="BlueFarTest2", group="Test")
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
        lp.waitMillis(500);
        /**
         * sense the correct key column on the pictograph using vuforia
         */
        int leftColumnBlue = 19;
        int centerColumnBlue = 21;
        int rightColumnBlue = 24;
        telemetry.addData("GlyphColumnKey:", "%s", glyphColumnKey );
        Boolean isJewelBlue = null;

        if (robot.servoJewel == null)
        {
            telemetry.addData("jewelservo", "null");
        }
        else {
            robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_JEWEL_OUT);
        }

        telemetry.update();
        /**
         * use the rev color sensor to detect jewel color and knock jewel off
         * try max 9 times, once see  the color of the jewel then exit.
         */
       /* try {
            Bitmap bitmap = robot.relicRecoveryVuMark.vuforia.getBitmap();
            Mat origimg = new Mat(bitmap.getHeight(), bitmap.getWidth(), CvType.CV_8UC4);   // 8 bits  3 channel
            Utils.bitmapToMat(bitmap, origimg);

            JewelDetector jd = new JewelDetector();
            RevColorDistanceSensor.COLORTYPE jewel = jd.GetJewelColor(origimg);
            telemetry.addData("REVAMPED_JEWEL", "%s", jewel);
            //jd.processFrame(origimg, true);
            //JewelDetector.JewelOrder order = jd.getCurrentOrder();
            //telemetry.addData("DOGCV_JEWEL", "%s", order);

            RobotLog.vv("JEWEL", "%s ", jewel);
            // for fun,  detect the hexgons.
            jd.getHexgonPattern(origimg);
            origimg.release();
        } catch(Exception e) {
            RobotLog.e("Vuforia " + e.getMessage());
        }*/
        for (int i = 1; i < 8; i++) {

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
            telemetry.addData("SERVO", "%5.2f", RobotRevAmpedConstants.SERVO_JEWEL_OUT - i*3/255f);
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
        telemetry.update();
        robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_JEWEL_IN);
        robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_DETECT);
        lp.waitMillis(300);

        drive.moveToEncoderInch(TurnType.STRAFE,
                38,
                0.8f,
                3000,
                true,
                false);
        float startHeading = robot.gyroSensor.getHeading();
        while(startHeading < -1.5 || startHeading > 1.5) {
            if (startHeading < -1.5) {
                drive.setPower(-0.4f, 0.4f);
            }else if (startHeading > 1.5) {
                drive.setPower(0.4f, -0.4f);
            }
            startHeading = robot.gyroSensor.getHeading();
            telemetry.addData("heading", startHeading);
            telemetry.update();
        }
        drive.turn(-150,
                0.7f,
                4000,
                true);
        robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
        robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
        Boolean FirstGlyph = false;
        drive.moveToTime(0.5f, 400);
        lp.waitMillis(100);
        int countTray = glyphCount.GetGlyphCount(0);
        if (countTray!=2) {
            FirstGlyph = true;
            long timestamp = System.currentTimeMillis();
            drive.moveToTime(0.5f, 300);
            lp.waitMillis(100);
            countTray = glyphCount.GetGlyphCount(0);
            while (countTray != 2) {
                robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                lp.waitMillis(100);
                countTray = glyphCount.GetGlyphCount(0);
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
        drive.turn(-22,
                0.4f,
                2000,
                false);
        drive.moveToEncoderInch(TurnType.FORWARD,
                -24,
                0.8f,
                3000,
                true,
                false);
        robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_IN);
        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
        robot.sweeperRight.setPower(0);
        robot.sweeperLeft.setPower(0);
        float sonarWallBlue = robot.sonarR.getDistance();

            if (glyphColumnKey==RelicRecoveryVuMark.LEFT) {
                if (sonarWallBlue < 19 ) {
                    telemetry.addLine("hi");
                    telemetry.update();
                    drive.moveToEncoderInch(TurnType.STRAFE,
                            17,
                            0.8f,
                            8000,
                            true,
                            false);
                    sonarWallBlue = robot.sonarR.getDistance();
                }
                //drive.moveToTime(-0.4f, 400);
                while (sonarWallBlue>19) {
                    drive.setPower(0.5f, -0.5f, -0.5f, 0.5f);
                    telemetry.addData("distance from wall", sonarWallBlue);
                    telemetry.update();
                     sonarWallBlue = robot.sonarR.getDistance();
                }
                drive.turn(25,
                        0.8f,
                        4000,
                        true
                );
            } else if (glyphColumnKey == RelicRecoveryVuMark.CENTER) {
                //drive.moveToTime(-0.35f, 400);
                if (sonarWallBlue < 21.5 ) {
                    telemetry.addLine("hi");
                    drive.moveToEncoderInch(TurnType.STRAFE,
                            14,
                            0.3f,
                            8000,
                            true,
                            false);
                    telemetry.update();
                    sonarWallBlue = robot.sonarR.getDistance();
                }
                while (sonarWallBlue>21.5) {
                    drive.setPower(0.5f, -0.5f, -0.5f, 0.5f);
                    telemetry.addData("distance from wall", sonarWallBlue);
                    telemetry.update();
                    sonarWallBlue = robot.sonarR.getDistance();
                }
                drive.turn(25,
                        0.6f,
                        4000,
                        true);
            } else {

                if (sonarWallBlue < 24) {
                    drive.moveToTime(-0.3f, 300);
                    drive.turn(20,
                            0.8f,
                            3000,
                            true);
                }
                else {
                    while (sonarWallBlue>24) {
                        drive.setPower(0.6f, -0.6f, -0.6f, 0.6f);
                        telemetry.addData("distance from wall", sonarWallBlue);
                        sonarWallBlue = robot.sonarR.getDistance();
                    }
                    drive.turn(18,
                            0.8f,
                            3000,
                            true);
                }

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
            drive.moveToTime(-0.5f, 500);
            lp.waitMillis(200);
            drive.moveToTime(0.5f, 500);
            robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
            /**
             *   for second and third glyphs
             */
            long farTime;
            farTime= System.currentTimeMillis();
            if (isGlyph &&farTime-startTime<17500 /*&& (glyphColumnKey==RelicRecoveryVuMark.RIGHT ||
                    glyphColumnKey==RelicRecoveryVuMark.LEFT || glyphColumnKey==RelicRecoveryVuMark.CENTER)*/) {


              if ((glyphColumnKey == RelicRecoveryVuMark.CENTER)/* || (glyphColumnKey==RelicRecoveryVuMark.LEFT)*/) {
                  drive.turn(-7,
                          0.5f,
                          5000,
                          false
                  );
              } else if (glyphColumnKey==RelicRecoveryVuMark.LEFT) {
                  drive.turn(-20,
                          0.5f,
                          5000,
                          false
                  );
                  drive.moveToEncoderInch(TurnType.STRAFE,
                          -7,
                          0.75f,
                          3000,
                          true, true);
                  drive.turn(12,
                          0.5f,
                          5000,
                          false
                  );
                  /*drive.setPower(-0.344f, 0.49f, 0.49f, -0.344f);
                  lp.waitMillis(600);*/
              } else {
                  drive.turn(
                          -7,
                          0.4f,
                          5000,
                          false);
              }

                /**
                 *   move into position to drive to the glyph pit using the range sensor
                 */

                /*if (glyphColumnKey==RelicRecoveryVuMark.RIGHT) {
                    drive.turn(30,
                            0.7f,
                            2000,
                            true);
                } else {
                    drive.turn(20,
                            0.7f,
                            2000,
                            true);
                }*/
                /*float sonarInch = (((robot.sonarRF.getDistance()-12.7f)*2) + 65)/2;*/
                Boolean MoreGlyphs = false;
                robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_INTAKE+50);
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
                if (countTrayFar !=2) {
                    long timePit2 = System.currentTimeMillis();
                    long endTimePit2 = timePit2;
                    while (countTrayFar != 2 && endTimePit2 < (timePit2+2000)) {
                         drive.moveToTime(0.45f, 350);
                         drive.setPower(-0.4f, 0.4f);
                         lp.waitMillis(250);
                         countTrayFar = glyphCount.GetGlyphCount(0);
                         endTimePit2 = System.currentTimeMillis();
                         i2++;
                    }
                    RobotLog.vv("number of tries" ,"%d", i2);
                    robot.sweeperRight.setPower(0.6f);
                    robot.sweeperLeft.setPower(-0.6f);
                    //reverse motion
                    for (int i = i2; i>0; i--) {
                        drive.moveToTime(-0.45f, 350);
                        drive.setPower(0.3f, -0.3f);
                        lp.waitMillis(100);
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
                /*if (countTrayFar!=2) {
                    drive.turn(-16,
                            0.5f,
                            3000,
                            false);
                    lp.waitMillis(300);
                    countTrayFar = glyphCount.GetGlyphCount(0);
                    if (countTrayFar!=2) {
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                10,
                                0.9f,
                                700,
                                true,
                                false);
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                -10,
                                0.9f,
                                700,
                                true,
                                true);
                        drive.turn(16,
                                0.5f,
                                3000,
                                false);
                        robot.sweeperLeft.setPower(0);
                        robot.sweeperRight.setPower(0);
                        drive.moveToTime(-0.45f, 600);
                        robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                        lp.waitMillis(300);

                    } else {
                        drive.turn(16,
                                0.8f,
                                3000,
                                true);
                        robot.sweeperLeft.setPower(0.4f);
                        robot.sweeperRight.setPower(0.4f);
                        drive.moveToTime(-0.45f, 600);
                        robot.sweeperLeft.setPower(0);
                        robot.sweeperRight.setPower(0);
                        robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                        MoreGlyphs = true;
                    }
                } else {
                    robot.sweeperLeft.setPower(0);
                    robot.sweeperRight.setPower(0);
                    drive.moveToEncoderInch(TurnType.FORWARD,
                            -10,
                            0.5f,
                            3000,
                            false,
                            true);
                    robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                    robot.sweeperLeft.setPower(0.4f);
                    robot.sweeperRight.setPower(0.4f);
                    MoreGlyphs = true;

                }*/
                robot.sweeperLeft.setPower(0);
                robot.sweeperRight.setPower(0);
                if (glyphColumnKey==RelicRecoveryVuMark.CENTER ||
                        glyphColumnKey ==RelicRecoveryVuMark.LEFT && MoreGlyphs) {
                    float heading = robot.gyroSensor.getHeading();
                    while (heading <-1 || heading>1) {
                        if (heading>1) {
                            drive.setPower(0.5f, -0.5f);
                        } else {
                            drive.setPower(-0.5f, 0.5f);
                        }
                        heading = robot.gyroSensor.getHeading();
                        telemetry.addData("heading", heading);
                        telemetry.update();
                    }
                    drive.turn(-15,
                            0.5f,
                            2000,
                            false);
                }
                robot.sweeperRight.setPower(0.6f);
                robot.sweeperLeft.setPower(0.6f);
                if (MoreGlyphs) {
                    drive.moveToEncoderInch(TurnType.FORWARD,
                            -24,
                            0.85f,
                            1800,
                            true,
                            true);
                } else {
                    drive.moveToEncoderInch(TurnType.FORWARD,
                            -32,
                            0.85f,
                            1800,
                            true,
                            true);
                }
                robot.sweeperRight.setPower(0);
                robot.sweeperLeft.setPower(0);
                lp.waitMillis(100);
                float sonarWallBlue2 = robot.sonarR.getDistance();
                if (glyphColumnKey==RelicRecoveryVuMark.LEFT) {
                    while (sonarWallBlue2>23) {
                        drive.setPower(0.5f, -0.5f, -0.5f, 0.5f);
                        telemetry.addData("distance from wall", sonarWallBlue2);
                        telemetry.update();
                        sonarWallBlue2 = robot.sonarR.getDistance();
                    }
                    robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_UP);
                    drive.turn(16,
                            0.5f,
                            3000,
                            false
                    );
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                    drive.moveToTime(-0.5f, 300);
                    robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
                    drive.moveToTime(-0.4f, 450);
                    lp.waitMillis(100);
                    drive.moveToTime(0.5f, 400);
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                } else if (glyphColumnKey==RelicRecoveryVuMark.CENTER) {
                    if (sonarWallBlue < 23) {
                        while (sonarWallBlue2 > 18) {
                            drive.setPower(0.5f, -0.5f, -0.5f, 0.5f);
                            telemetry.addData("distance from wall", sonarWallBlue2);
                            telemetry.update();
                            sonarWallBlue2 = robot.sonarR.getDistance();
                        }
                    } else {
                        while (sonarWallBlue2 > 23) {
                            drive.setPower(0.5f, -0.5f, -0.5f, 0.5f);
                            telemetry.addData("distance from wall", sonarWallBlue2);
                            telemetry.update();
                            sonarWallBlue2 = robot.sonarR.getDistance();
                        }
                    }
                    robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_UP);
                    drive.turn(16,
                            0.5f,
                            3000,
                            false
                    );
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                    drive.moveToTime(-0.45f, 300);
                    robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
                    drive.moveToTime(-0.3f, 450);
                    lp.waitMillis(100);
                    drive.moveToTime(0.5f, 400);
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                } else {
                    float heading = robot.gyroSensor.getHeading();
                    while (heading <-1 || heading>1) {
                        if (heading>1) {
                            drive.setPower(0.5f, -0.5f);
                        } else {
                            drive.setPower(-0.5f, 0.5f);
                        }
                        heading = robot.gyroSensor.getHeading();
                        telemetry.addData("heading", heading);
                        telemetry.update();
                    }
                    drive.turn(-15,
                            0.5f,
                            2000,
                            false);
                    sonarWallBlue2 = robot.sonarR.getDistance();
                    robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_IN);
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);

                   /* while (sonarWallBlue2>17) {
                        drive.setPower(0.5f, -0.5f, -0.5f, 0.5f);
                        telemetry.addData("distance from wall", sonarWallBlue2);
                        telemetry.update();
                        sonarWallBlue2 = robot.sonarR.getDistance();
                    }*/
                   drive.moveToEncoderInch(TurnType.STRAFE,
                           20,
                           0.8f,
                           5000,
                           true,
                           true);
                    robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_UP);
                    drive.turn(12,
                            0.5f,
                            3000,
                            false
                    );
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                    drive.moveToTime(-0.45f, 300);
                    robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
                    drive.moveToTime(-0.3f, 450);
                    lp.waitMillis(100);
                    drive.moveToTime(0.5f, 400);
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                }
                robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                drive.stop();
            }
        long timeStamp = System.currentTimeMillis();


        RobotLog.i("AutonomousRevamped finish in " + (System.currentTimeMillis() - startTime) + " milliseconds");
        lp.waitMillis(5000);
    }
}



