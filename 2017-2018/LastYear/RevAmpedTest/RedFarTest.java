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
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="RedFarTest2", group="Test")
public class RedFarTest
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

    //public VuforiaLocalizerImplSubClass vuforia = new VuforiaLocalizerImplSubClass(parameters);

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
            if (wallDistanceL>20.5 && wallDistanceL<21.5) {
                telemetry.addLine("Aligned....Sai!!!!!!!!!!!");
                robot.ledGreen.on();
            }
            telemetry.update();
            if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {
                break;
            }
        }
        SelectLinear sl = new SelectLinear(this);
        //boolean isRed = sl.selectAlliance();
        //boolean isNear = sl.selectPosition();
        boolean isGlyph = sl.extraGlyph();
        //int waitMillis = sl.adjustDelay();
        //telemetry.addData("Alliance", isRed ? "Red" : "Blue");
        //telemetry.addData("Position", isNear ? "Near" : "Far");
        telemetry.addData("Extra Glyph?", isGlyph ? "Yes" : "No");
        telemetry.addData("Waiting", "AutonomousRevAmped");
        telemetry.update();
        /**
         * if the alliance is red, then show red led. if it is blue, then show blue led
         */

            robot.ledRed.on();

        WaitLinear lp = new WaitLinear(this);
        // lp.waitMillis(waitMillis);


        /**
         * force the enocder of the slide to 0. Reset the heading of the gyro sensor
         * make the container flat so we can put a glyph in at the start of autonomous
         */
        robot.motorSlide.resetPosition();
        robot.gyroSensor.resetHeading();
        //wait for start
        waitForStart();
        //autonomous program begin
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
        robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_INTAKE+15);
        robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_JEWEL_OUT);
        RelicRecoveryVuMark glyphColumnKey = robot.relicRecoveryVuMark.getGlyphColumnKey();
        lp.waitMillis(500);
        int leftColumnBlue = 47;
        int centerColumnBlue = 57;
        int rightColumnBlue = 64;
        /**
         * sense the correct key column on the pictograph using vuforia
         */
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
         * try max 6 times, once see  the color of the jewel then exit.
         */
        /*try {
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


        for (int i = 1; i < 6; i++) {

            lp.waitMillis(50);

            RevColorDistanceSensor.COLORTYPE jewelcolor = robot.revColorDistanceSensor.getColor();

            if (jewelcolor == RevColorDistanceSensor.COLORTYPE.RED )
            {

                    robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_HIT_LEFT);
                break;
            }
            else if (jewelcolor == RevColorDistanceSensor.COLORTYPE.BLUE)
            {

                    robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_HIT_RIGHT);
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
        lp.waitMillis(300);
        robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_JEWEL_IN);
        robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_DETECT);

        drive.moveToEncoderInch(TurnType.STRAFE,
                36,
                0.8f,
                3000,
                true,
                false);

        drive.turn(-12,
                0.4f,
                3000,
                false);
        robot.sweeperRight.setPower(0.6f);
        robot.sweeperLeft.setPower(0.6f);
        drive.moveToTime(0.45f, 900);
        int countTray = glyphCount.GetGlyphCount(0);
        long startTimestamp = System.currentTimeMillis();
        long endTimestamp = startTimestamp;

        //int i = 0;
        while (countTray!=2 && endTimestamp<(1000+startTimestamp)) {

            robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
            robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
            countTray = glyphCount.GetGlyphCount(0);
            endTimestamp = System.currentTimeMillis();
            /*lp.waitMillis(200);

            drive.moveToTime(0.4f, 200);
            i++;*/
        }
        drive.moveToTime(-0.5f, 900);
        //smooth out glyphs inside by intaking one wheel and outaking one wheel
        robot.sweeperRight.setPower(0.6f);
        robot.sweeperLeft.setPower(-0.6f);
        //correct the heading
        drive.turn(8,
                0.4f,
                3000,
                false);
        float startHeading = robot.gyroSensor.getHeading();
            while (startHeading < -1.5 || startHeading > 1.5) {
                if (startHeading < -1.5) {
                    drive.setPower(-0.3f, 0.3f);
                } else if (startHeading > 1.5) {
                    drive.setPower(0.3f, -0.3f);
                }
                startHeading = robot.gyroSensor.getHeading();
                RobotLog.vv("heading", "%5.2f", startHeading);
            }
        robot.sweeperRight.setPower(0);
        robot.sweeperLeft.setPower(0);
       /* drive.turn(Math.round(-startHeading),
                0.4f,
                2000,
                false);*/
        robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
        drive.moveToEncoderInch(TurnType.FORWARD,
                -25,
                1f,
                3000,
                true,
                false);
        //clamp glyphs and set dumper to prepare for dumping
        robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_IN);
        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
        float sonarWallRed = robot.sonarL.getDistance();
        RobotLog.vv("distance from wall start", "%5.2f", sonarWallRed);
        if (glyphColumnKey==RelicRecoveryVuMark.RIGHT) {
            //if reading is already less than 45 the reading is invalid, strafe 400 milliseconds then read again
            if (sonarWallRed<47) {
                //left back wheel has more power to account for weight distribution
                drive.setPower(-0.3f, 0.4f, 0.3f, -0.3f);
                lp.waitMillis(400);
            }
            sonarWallRed = robot.sonarL.getDistance();
            while (sonarWallRed>47) {
                drive.setPower(-0.35f, 0.45f, 0.35f, -0.35f);
                telemetry.addData("distance from wall", sonarWallRed);
                telemetry.update();
                sonarWallRed = robot.sonarL.getDistance();
            }
            drive.turn(-20,
                    0.6f,
                    3000,
                    true);
        } else if (glyphColumnKey == RelicRecoveryVuMark.CENTER) {
            if (sonarWallRed<58) {
                drive.setPower(-0.35f, 0.45f, 0.35f, -0.35f);
                lp.waitMillis(150);
            }
            sonarWallRed = robot.sonarL.getDistance();
            while (sonarWallRed>58) {

                drive.setPower(-0.35f, 0.45f, 0.35f, -0.35f);
                telemetry.addData("distance from wall", sonarWallRed);
                telemetry.update();
                sonarWallRed = robot.sonarL.getDistance();
            }
            drive.turn(-21,
                    0.6f,
                    3000,
                    true);

        } else {
           while (sonarWallRed>64) {
                drive.setPower(-0.3f, 0.4f, 0.3f, -0.3f);
                telemetry.addData("distance from wall", sonarWallRed);
                telemetry.update();
               sonarWallRed = robot.sonarL.getDistance();
                //sonarWallRed = robot.sonarL.getDistance();
            }
            drive.turn(-12,
                    0.6f,
                    3000,
                    true);

        }
        /**
         *  dump the first glyph
         */
        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
        drive.moveToTime(-0.5f, 650);
        robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
        lp.waitMillis(200);
        drive.moveToTime(0.5f, 500);
        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
        /**
         *   for second and third glyphs
         */
        long farTime;
        farTime= System.currentTimeMillis();
        if (isGlyph &&farTime-startTime<15000 /*&& (glyphColumnKey==RelicRecoveryVuMark.RIGHT ||
                glyphColumnKey==RelicRecoveryVuMark.LEFT || glyphColumnKey==RelicRecoveryVuMark.CENTER)*/) {

            /*if ((glyphColumnKey==RelicRecoveryVuMark.LEFT && !isRed) ||
                    (glyphColumnKey==RelicRecoveryVuMark.RIGHT && isRed)) {
                drive.turn(isRed ? -8 : 24,
                        0.6f,
                        4000,
                        true);
            } else if (glyphColumnKey==RelicRecoveryVuMark.CENTER){
                drive.turn(isRed ? -24 : 25,
                        0.6f,
                        4000,
                        true);
            } else {
                drive.turn(isRed ? -26 : 30,
                        0.6f,
                        4000,
                        true);
            }*/
            /*if (glyphColumnKey ==RelicRecoveryVuMark.CENTER || glyphColumnKey == RelicRecoveryVuMark.RIGHT) {
                drive.turn(9,
                        0.7f,
                        2000,
                        true);
            }*/


            /**
             *   move into position to drive to the glyph pit using the range sensor
             */
           /* if (glyphColumnKey == RelicRecoveryVuMark.CENTER || glyphColumnKey == RelicRecoveryVuMark.RIGHT) {
                drive.turn(10,
                        0.5f,
                        3000,
                        false);
            }
            if (glyphColumnKey==RelicRecoveryVuMark.CENTER ) {
                drive.moveToEncoderInch(TurnType.STRAFE,
                        0,
                        0.75f,
                        3000,
                        true,
                        false);
            }else if (glyphColumnKey==RelicRecoveryVuMark.RIGHT){

                drive.moveToEncoderInch(TurnType.STRAFE,
                        7,
                        0.75f,
                        3000,
                        true,
                        false);
            }*/
           if (glyphColumnKey==RelicRecoveryVuMark.RIGHT) {
               drive.turn(16,
                       0.5f,
                       3000,
                       false);
               drive.moveToEncoderInch(TurnType.STRAFE,
                       7,
                       0.6f,
                       3000,
                       false,
                       false);
               drive.turn(-16,
                       0.5f,
                       3000,
                       false);
               /*drive.setPower(0.45f, 0.211f, 0.211f, 0.45f);
               lp.waitMillis(600);*/
           }


            /*if ((glyphColumnKey==RelicRecoveryVuMark.RIGHT&&isRed) ||
                    (glyphColumnKey==RelicRecoveryVuMark.LEFT && !isRed)) {
                drive.turn(isRed? -26:30,
                        0.7f,
                        2000,
                        true);
            } else  if ( glyphColumnKey==RelicRecoveryVuMark.CENTER){
                drive.turn(isRed? -26:36,
                        0.6f,
                        2000,
                        true);
            } else {
                drive.turn(isRed? -26:36,
                        0.7f,
                        2000,
                        true);
            }*/
            float sonarInch = 0.8f*robot.sonarRF.getDistance()+28.8f;
            RobotLog.vv("distance to glyphs","%5.1f" , sonarInch);
            telemetry.addData("distance to glyphs", sonarInch);
            telemetry.update();
            if (sonarInch>45) {
                sonarInch = 45;
            }
            robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_INTAKE+15);
            robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
            robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
            drive.moveToEncoderInch(TurnType.FORWARD,
                    Math.round(sonarInch),
                    0.8f,
                    2400,
                    true,
                    true);
            Boolean MoreGlyphs = false;
            lp.waitMillis(400);
            int countTrayFar = glyphCount.GetGlyphCount(0);
            int i2 = 0;
            if (countTrayFar !=2) {
                long timePit2 = System.currentTimeMillis();
                long endTimePit2 = timePit2;
                while (countTrayFar != 2 && endTimePit2 < (timePit2+3000)) {
                    drive.moveToTime(0.45f, 350);
                    drive.setPower(0.4f, -0.4f);
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
                    drive.setPower(-0.4f, 0.4f);
                    lp.waitMillis(250);
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
                robot.sweeperRight.setPower(0.6f);
                robot.sweeperLeft.setPower(-0.6f);
                drive.turn(-16, 0.4f, 3000, false);
                MoreGlyphs = false;
                robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
            }
            /*int countTrayFar = glyphCount.GetGlyphCount(0);
            if (countTrayFar!=2) {
                telemetry.addData("saw", countTrayFar);
                telemetry.update();
                lp.waitMillis(100);*/
                /**
                 *  go in twice to get two glyphs. also reverse intake briefly to spit out clogged glyphs
                 */
                /*robot.sweeperLeft.setPower(-0.7f);
                robot.sweeperRight.setPower(-0.7f);
                lp.waitMillis(300);
                robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                lp.waitMillis(50);

                drive.turn(-15,
                        0.8f,
                        3000,
                        true);
                lp.waitMillis(400);
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
                    drive.turn(15,
                            0.8f,
                            3000,
                            true);

                    robot.sweeperLeft.setPower(0);
                    robot.sweeperRight.setPower(0);
                    drive.moveToTime(-0.45f, 600);
                    robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                    lp.waitMillis(300);

                } else {
                    drive.turn(15,
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
                drive.moveToTime(-0.45f, 600);
                robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                robot.sweeperLeft.setPower(0.4f);
                robot.sweeperRight.setPower(0.4f);
                MoreGlyphs = true;

            }*/
            /*drive.moveToEncoderInch(TurnType.FORWARD,
                    -10,
                    0.5f,
                    3000,
                    false,
                    false);*/
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
            drive.turn(14,
                    0.5f,
                    5000,
                    false);
            telemetry.update();
            robot.sweeperLeft.setPower(0);
            robot.sweeperRight.setPower(0);
            drive.moveToEncoderInch(TurnType.FORWARD,
                    20-Math.round(sonarInch),
                    0.8f,
                    2000,
                    true,
                    false);
            if (glyphColumnKey==RelicRecoveryVuMark.RIGHT) {
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
                robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_IN);
                robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_UP);
                lp.waitMillis(100);

                float sonarWallRed2 = robot.sonarL.getDistance();
                if (sonarWallRed2 > 64) {
                    while (sonarWallRed2 > 64) {
                        drive.setPower(-0.35f, 0.45f, 0.35f, -0.35f);
                        telemetry.addData("distance from wall", sonarWallRed2);
                        telemetry.update();
                        sonarWallRed2 = robot.sonarL.getDistance();

                    }
                } else {
                    while (sonarWallRed2 < 64) {
                        drive.setPower(0.35f, -0.45f, -0.35f, 0.35f);
                        telemetry.addData("distance from wall", sonarWallRed2);
                        telemetry.update();
                        sonarWallRed2 = robot.sonarL.getDistance();
                    }
                }
                drive.turn(-12,
                        0.6f,
                        3000,
                        true);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                drive.moveToTime(-0.5f, 650);
                robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
                lp.waitMillis(200);
                drive.moveToTime(0.5f, 500);
            } else if (glyphColumnKey == RelicRecoveryVuMark.CENTER) {
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
                robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_IN);
                robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_UP);

                float sonarWallRed2 = robot.sonarL.getDistance();
                lp.waitMillis(100);

                if (sonarWallRed2 > 64) {
                    while (sonarWallRed2 > 64) {
                        drive.setPower(-0.35f, 0.45f, 0.35f, -0.35f);
                        telemetry.addData("distance from wall", sonarWallRed2);
                        telemetry.update();
                        sonarWallRed2 = robot.sonarL.getDistance();

                    }
                } else {
                    while (sonarWallRed2 < 64) {
                        drive.setPower(0.35f, -0.45f, -0.35f, 0.35f);
                        telemetry.addData("distance from wall", sonarWallRed2);
                        telemetry.update();
                        sonarWallRed2 = robot.sonarL.getDistance();
                    }
                }
                drive.turn(-12,
                        0.6f,
                        3000,
                        true);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                drive.moveToTime(-0.5f, 650);
                robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
                lp.waitMillis(200);
                drive.moveToTime(0.5f, 500);
            }else {
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
                robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_IN);
                /*drive.turn(12,
                        0.4f,
                        3000,
                        false);*/
                lp.waitMillis(100);
                float sonarWallRed2 = robot.sonarL.getDistance();
                while (sonarWallRed2>48) {
                    drive.setPower(-0.4f, 0.5f, 0.4f, -0.4f);
                    telemetry.addData("distance from wall", sonarWallRed2);
                    telemetry.update();
                    sonarWallRed2 = robot.sonarL.getDistance();

                }
                drive.turn(-20,
                        0.6f,
                        3000,
                        true);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                drive.moveToTime(-0.5f, 650);
                robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
                lp.waitMillis(200);
                drive.moveToTime(0.5f, 500);
            }
            robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
            robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
            robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_IN);
            drive.stop();

            /**
             *    go back to the cryptobox
             */
            /*if ((glyphColumnKey==RelicRecoveryVuMark.LEFT && isRed) ||
                    (glyphColumnKey==RelicRecoveryVuMark.RIGHT && !isRed) ) {
                drive.moveToEncoderInch(TurnType.FORWARD,
                        -33,
                        1f,
                        3000,
                        true,
                        true);
            } else {
                drive.moveToEncoderInch(TurnType.FORWARD,
                        -42,
                        1f,
                        3000,
                        true,
                        true);
            }*/

           /* if ((glyphColumnKey==RelicRecoveryVuMark.LEFT && isRed) ||
                    (glyphColumnKey==RelicRecoveryVuMark.RIGHT && !isRed) && isGlyph) {

                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
                //robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_OUT);
                if (isRed) {
                    drive.turn(12,
                            0.6f,
                            3000,
                            false);
                    drive.moveToTime(-0.2f, 300);
                } else {
                    drive.setPower(0.6f, 0.6f, -0.6f, -0.6f);
                    lp.waitMillis(450);
                }

                drive.setPower(0);

                drive.moveToEncoderInch(TurnType.STRAFE,
                        isRed? -21:24,
                        0.7f,
                        2000,
                        false,
                        true);
                robot.slide.moveToEncoder(1000);
                if (!isRed) {

                    drive.moveToTime(-0.45f, 300);
                }
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                if (isRed) {
                    drive.moveToTime(-0.4f, 300);
                    drive.moveToTime(0.3f, 300);
                    drive.moveToTime(-0.4f, 300);

                }
                drive.moveToTime(0.4f, 400);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                robot.slide.moveToEncoder(75);
                drive.moveToEncoderInch(TurnType.STRAFE,
                        8,
                        0.7f,
                        3000,
                        true,
                        true);

            }else if ((glyphColumnKey==RelicRecoveryVuMark.RIGHT && isRed) ||
                    (glyphColumnKey==RelicRecoveryVuMark.LEFT && !isRed) && isGlyph) {
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
                //  robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_OUT);

                if (isRed) {
                    drive.moveToTime(0.35f, 300
                    );
                }
                drive.turn(isRed? 20:-4,
                        0.7f,
                        2000,
                        false);
                drive.moveToEncoderInch(TurnType.FORWARD,
                        isRed?-6:0,
                        0.8f,
                        2000,
                        true,
                        true);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP-5);
                drive.moveToTime(-0.4f, 700);
                drive.moveToTime(0.4f, 500);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);

                lp.waitMillis(200);
                drive.moveToEncoderInch(TurnType.STRAFE,
                        isRed?-5:10,
                        0.8f,
                        2000,
                        true,
                        true);


            } else if (glyphColumnKey==RelicRecoveryVuMark.CENTER&& isGlyph){
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
                //robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_OUT);

                if (isRed) {
                    if (!MoreGlyphs) {
                        drive.moveToTime(0.4f, 300);
                    } else {
                        drive.moveToTime(0.3f, 250);
                    }

                    drive.turn(MoreGlyphs?12:12, 0.6f, 2000, false);

                }
                if (!isRed && MoreGlyphs) {
                    drive.moveToTime(0.4f, 450);
                    drive.turn(12, 0.6f, 2000, true);
                }
                if (isRed||(!isRed && MoreGlyphs)) {
                    drive.moveToEncoderInch(TurnType.STRAFE,
                            isRed ? MoreGlyphs?0:-14     : -9,
                            0.8f,
                            3000,
                            true,
                            true);
                }
                robot.slide.moveToEncoder(1000);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                if (isRed) {

                    drive.moveToTime(-0.45f, 600);
                } else {

                    drive.moveToTime(-0.45f, 800);
                }

                drive.moveToTime(0.4f, 600);

                drive.turn(isRed?0:-12,
                        0.6f,
                        2000,
                        false);
                robot.slide.moveToEncoder(75);

                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);

                drive.moveToEncoderInch(TurnType.STRAFE,
                        isRed?MoreGlyphs?-12:5:18,
                        0.8f,
                        2000,
                        true,
                        true);


            }*/
            robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
        }
        long timeStamp = System.currentTimeMillis();


        RobotLog.i("AutonomousRevamped finish in " + (System.currentTimeMillis() - startTime) + " milliseconds");
        lp.waitMillis(5000);
    }
}



