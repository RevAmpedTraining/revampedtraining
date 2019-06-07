package org.firstinspires.ftc.teamcode.LastYear;

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
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutonomousRevamped2", group="Game")
public class AutonomousRevamped2
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
            }
            else {
                robot.ledBlue.on();
            }
            WaitLinear lp = new WaitLinear(this);
            // lp.waitMillis(waitMillis);
            RelicRecoveryVuMark glyphColumnKey = robot.relicRecoveryVuMark.getGlyphColumnKey();
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

            int Offset = 0;
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
            for (int i = 1; i < 10; i++) {

                lp.waitMillis(100);

                RevColorDistanceSensor.COLORTYPE jewelcolor = robot.revColorDistanceSensor.getColor();

                if (jewelcolor == RevColorDistanceSensor.COLORTYPE.RED )
                {
                    isJewelBlue = Boolean.FALSE;
                    robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_JEWEL_OUT);
                    if (isRed) {
                        robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_HIT_LEFT);
                    } else {
                        robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_HIT_RIGHT);

                    }
                    break;
                }
                else if (jewelcolor == RevColorDistanceSensor.COLORTYPE.BLUE)
                {
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

                robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_JEWEL_OUT - i*3/255f);
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
            lp.waitMillis(200);
            robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_JEWEL_IN);
            robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_DETECT);
            float startHeading;
            startHeading = robot.gyroSensor.getHeading();
            /**
             * for blue or red and near
             */

            if (isNear) {
                /**
                 * for blue and near: move into position to deposit the first glyph
                 */
                if (!isRed) {
                    if (glyphColumnKey==RelicRecoveryVuMark.LEFT) {
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                48,
                                0.8f,
                                5000,
                                true,
                                true);
                        drive.turn(-50, 0.8f, 2000, true);
                    } else if (glyphColumnKey==RelicRecoveryVuMark.CENTER) {
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                57,
                                0.75f,
                                5000,
                                true,
                                true);
                        drive.turn(-48, 0.6f, 2000, false);
                    } else {
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                40,
                                0.75f,
                                5000,
                                true,
                                true);
                        drive.turn(-106, 0.65f, 6000, false);
                    }
                    /**
                     *  red and near: move into position to deposit the first glyph
                     */

                } else {
                    if (glyphColumnKey==RelicRecoveryVuMark.RIGHT) {
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                -50,
                                0.75f,
                                5000,
                                true,
                                true);
                        drive.turn(-104, 0.7f, 2000, false);
                    } else if (glyphColumnKey==RelicRecoveryVuMark.CENTER) {
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
                }
                lp.waitMillis(100);
                /**
                 * dump the glyph and push in
                 */
                if (isRed &&glyphColumnKey==RelicRecoveryVuMark.RIGHT) {
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
                telemetry.addData("time", nearTime-startTime);
                telemetry.update();
                if ( isGlyph&&nearTime-startTime<15000 && (glyphColumnKey==RelicRecoveryVuMark.RIGHT ||
                        glyphColumnKey==RelicRecoveryVuMark.LEFT || glyphColumnKey==RelicRecoveryVuMark.CENTER)) {
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                    lp.waitMillis(300);
                    //correction of heading so that we can go into the glyph pit straight
                    if (glyphColumnKey==RelicRecoveryVuMark.RIGHT ) {
                        drive.turn(isRed? 28:24,
                                0.35f,
                                2000,
                                false);
                    } else if (glyphColumnKey==RelicRecoveryVuMark.CENTER){
                        drive.turn(-35,
                                0.35f,
                                2000,
                                false);
                    } else {
                        drive.turn(isRed? -30:-30,
                                0.35f,
                                2000,
                                false);
                    }
                    /**
                     *go to the glyph pit
                     */
                    float sonarCm2;
                    sonarCm2 = (robot.sonarRF.getDistance()-12.7f)*2+72;
                    telemetry.addData("sonar distance", sonarCm2);
                    telemetry.update();
                    robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_OUT);
                    robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                    robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                    float startEncoderValue = drive.getEncoder();
                    drive.moveToEncoderInch(TurnType.FORWARD,
                            Math.round(sonarCm2/2.54f),
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
                    if (countTray !=2 ) {
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
                        if (countTray !=2 ) {
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
                        }
                        else {
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
                    while (heading <-2 || heading>2) {
                        if (heading>2) {
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
                            32-Math.round(sonarCm2/2.54f),
                            0.8f,
                            3000,
                            true,
                            true
                    );
                    if (glyphColumnKey==RelicRecoveryVuMark.LEFT) {
                        if (isRed   ) {
                            drive.moveToEncoderInch(TurnType.STRAFE,
                                    -9, 0.5f, 2000, false, true);
                        }
                        drive.turn(isRed?-30:-30,
                                0.4f,
                                2000,
                                false);
                        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
                       // robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_OUT);

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

                    } else if (glyphColumnKey==RelicRecoveryVuMark.RIGHT) {
                        if (isRed) {
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
                        } else {
                            drive.moveToEncoderInch(TurnType.STRAFE,
                                    5,
                                    0.6f,
                                    2000,
                                    false,
                                    true);
                            drive.turn(12,
                                    0.5f,
                                    2000,
                                    false);
                        }
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
                    } else if (glyphColumnKey==RelicRecoveryVuMark.CENTER) {
                        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
                        //robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_OUT);

                        if (!isRed) {
                            drive.moveToEncoderInch(TurnType.STRAFE,
                                    -5,
                                    0.8f,
                                    2000,
                                    true,
                                    false);
                        }
                        drive.turn(isRed?-14:-12,
                                0.3f,
                                2000,
                                false);
                        if (!isRed) {
                            robot.slide.moveToEncoder(1000);
                        }

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

            /**
             *            for blue or red and far
             */
            if (!isNear) {
                float headingFar = robot.gyroSensor.getHeading();
                int strafeFar = 15;
                lp.waitMillis(100);
                drive.moveToEncoderInch(TurnType.FORWARD,
                        isRed ? -35 : 35,
                        0.8f,
                        2500,
                        true,
                        false);
                /**
                 *drive and turn differently based on vumark pictograph
                 */

                if ((glyphColumnKey==RelicRecoveryVuMark.LEFT && !isRed )||
                        ( glyphColumnKey==RelicRecoveryVuMark.RIGHT && isRed)) {
                    drive.turn(isRed?16:154,
                            0.6f,
                            3000,
                            true);
                } else if (glyphColumnKey == RelicRecoveryVuMark.CENTER) {
                    drive.moveToEncoderInch(TurnType.STRAFE,
                            isRed?4:3,
                            0.6f,
                            1000,
                            true,
                            true);
                    drive.turn(isRed?27:142,
                            0.6f,
                            3000,
                            true);
                } else {
                    drive.moveToEncoderInch(TurnType.STRAFE,
                            isRed?15:14,
                            0.8f,
                            3000,
                            true, false);
                    RobotLog.vv("far strafing", "%d", strafeFar);
                    drive.turn(isRed?30: 144,
                            0.8f,
                            3000,
                            true
                    );
                }
                /**
                 *  dump the first glyph
                 */
                if (!isRed && glyphColumnKey==RelicRecoveryVuMark.RIGHT) {
                    drive.moveToTime(-0.4f, 300);
                }
                drive.moveToTime(-0.35f, 350);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DUMP);
                lp.waitMillis(800);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                drive.moveToTime(0.5f, 250);
                drive.moveToTime(-0.5f, 500);
                drive.moveToTime(0.35f, 300);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                /**
                 *   for second and third glyphs
                 */
                long farTime;
                farTime= System.currentTimeMillis();
                if (isGlyph &&farTime-startTime<14000 && (glyphColumnKey==RelicRecoveryVuMark.RIGHT ||
                        glyphColumnKey==RelicRecoveryVuMark.LEFT || glyphColumnKey==RelicRecoveryVuMark.CENTER)) {

                    if ((glyphColumnKey==RelicRecoveryVuMark.LEFT && !isRed) ||
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
                    }
                    drive.moveToEncoderInch(TurnType.FORWARD,
                            5,
                            0.75f,
                            1000,
                            true,
                            true);
                    /**
                     *   move into position to drive to the glyph pit using the range sensor
                     */
                    if (glyphColumnKey==RelicRecoveryVuMark.CENTER) {
                        drive.moveToEncoderInch(TurnType.STRAFE,
                                isRed?18:-21,
                                0.75f,
                                3000 ,
                                true,
                                false);
                    } else if ((glyphColumnKey==RelicRecoveryVuMark.RIGHT&&isRed) ||
                            (glyphColumnKey==RelicRecoveryVuMark.LEFT && !isRed)) {
                        drive.moveToEncoderInch(TurnType.STRAFE,
                                isRed? 22:-25,
                                0.65f,
                                3000,
                                true, false);
                    } else {
                        drive.moveToEncoderInch(TurnType.STRAFE,
                                isRed? 9:-6,
                                0.65f,
                                3000,
                                true,
                                false);
                    }
                    if ((glyphColumnKey==RelicRecoveryVuMark.RIGHT&&isRed) ||
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
                    }

                    robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_OUT);
                    robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                    robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                    drive.moveToEncoderInch(TurnType.FORWARD,
                            52,
                            1f,
                            4000,
                            true,
                            true);
                    Boolean MoreGlyphs = false;
                    lp.waitMillis(1000);
                    int countTrayFar = glyphCount.GetGlyphCount(0);
                    if (countTrayFar!=2) {
                        telemetry.addData("saw", countTrayFar);
                        telemetry.update();
                        lp.waitMillis(100);
                        /**
                         *  go in twice to get two glyphs. also reverse intake briefly to spit out clogged glyphs
                         */
                        robot.sweeperLeft.setPower(-0.7f);
                        robot.sweeperRight.setPower(-0.7f);
                        lp.waitMillis(450);
                        robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                        robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                        lp.waitMillis(100);

                        drive.turn(-15,
                                0.8f,
                                3000,
                                true);
                        lp.waitMillis(500);
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
                            drive.moveToTime(-0.35f, 500);
                            robot.sweeperLeft.setPower(0);
                            robot.sweeperRight.setPower(0);
                            robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                            MoreGlyphs = true;
                        }
                    } else {
                        robot.sweeperLeft.setPower(0);
                        robot.sweeperRight.setPower(0);
                        drive.moveToTime(-0.35f, 500);
                        robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                        robot.sweeperLeft.setPower(0.4f);
                        robot.sweeperRight.setPower(0.4f);
                        MoreGlyphs = true;

                    }
                    float heading = robot.gyroSensor.getHeading();
                    while (heading <-2 || heading>2) {
                        if (heading>2) {
                            drive.setPower(0.5f, -0.5f);
                        } else {
                            drive.setPower(-0.5f, 0.5f);
                        }
                        heading = robot.gyroSensor.getHeading();
                        telemetry.addData("heading", heading);
                        telemetry.update();
                    }
                    robot.sweeperLeft.setPower(0);
                    robot.sweeperRight.setPower(0);
                    /**
                     *    go back to the cryptobox
                     */
                    if ((glyphColumnKey==RelicRecoveryVuMark.LEFT && isRed) ||
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
                    }

                    if ((glyphColumnKey==RelicRecoveryVuMark.LEFT && isRed) ||
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


                    }
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                }
            }
            long timeStamp = System.currentTimeMillis();


            RobotLog.i("AutonomousRevamped finish in " + (System.currentTimeMillis() - startTime) + " milliseconds");
            lp.waitMillis(5000);
        }
    }


