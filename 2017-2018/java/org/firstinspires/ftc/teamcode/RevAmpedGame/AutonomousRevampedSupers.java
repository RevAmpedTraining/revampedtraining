package org.firstinspires.ftc.teamcode.RevAmpedGame;

import com.qualcomm.robotcore.robot.Robot;
import com.revAmped.components.Button;
import com.revAmped.components.HwSonarAnalog;
import com.revAmped.components.TurnType;
import com.revAmped.config.RobotRevAmpedConstants;
import com.revAmped.linear.components.MecanumDriveLinear;
import com.revAmped.linear.components.RobotRevAmpedLinear2;
import com.revAmped.linear.util.SelectLinear;
import com.revAmped.linear.util.WaitLinear;
import com.revAmped.sensors.RevColorDistanceSensor;
import com.revAmped.sensors.VuMarkSensing;
import com.revAmped.util.GlyphDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
/**
 * Autonomous Program for Revamped
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutonomousRevamped Super", group="Game")
public class AutonomousRevampedSupers
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
            robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_IN);
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

            robot.ledGreen.set(isNear);
            robot.drawLed();
            /**
             * force the enocder of the slide to 0. Reset the heading of the gyro sensor
             * make the container flat so we can put a glyph in at the start of autonomous
             */
           // VuMarkSensing vuSensor = new VuMarkSensing(this.hardwareMap);
           // Boolean bInit = vuSensor.initialize();
            robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_IN);
            robot.motorSlide.resetPosition();
            robot.gyroSensor.resetHeading();
            //waiting for the start button
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

            robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
            robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_IN);
            robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_JEWEL_OUT);
            RelicRecoveryVuMark glyphColumnKey = robot.relicRecoveryVuMark.getGlyphColumnKey();
            lp.waitMillis(800);
            /**
             * sense the correct key column on the pictograph using vuforia
             */
            telemetry.addData("GlyphColumnKey:", "%s", glyphColumnKey );
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
             * try max 5 times, once see  the color of the jewel then exit.
             */
            for (int i = 1; i < 6; i++) {

                lp.waitMillis(100);

                RevColorDistanceSensor.COLORTYPE jewelcolor = robot.revColorDistanceSensor.getColor();

                if (jewelcolor == RevColorDistanceSensor.COLORTYPE.RED )
                {


                    if (isRed) {
                        robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_HIT_LEFT);
                    } else {
                        robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_HIT_RIGHT);

                    }
                    break;
                }
                else if (jewelcolor == RevColorDistanceSensor.COLORTYPE.BLUE)
                {


                    if (isRed) {
                        robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_HIT_RIGHT);
                    } else {
                        robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_HIT_LEFT);
                    }
                    break;
                }
                telemetry.addData("JEWEL", "%s", jewelcolor);
                // send the info back to driver station using telemetry function.

                robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_JEWEL_OUT - i*2/255f);
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
            lp.waitMillis(350);
            robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_JEWEL_IN);
            robot.servoJewelHit.setPosition(RobotRevAmpedConstants.SERVO_DETECT);
            Boolean MoreGlyphs = false;
            /**
             * for blue or red and near
             */
            if (isNear) {
                /**
                 * for blue and near: move into position to deposit the first glyph
                 */
                robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_IN);
                lp.waitMillis(200);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
                if (!isRed) {
                    if (glyphColumnKey==RelicRecoveryVuMark.LEFT) {
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                36,
                                0.8f,
                                5000,
                                true,
                                true);
                        drive.turn(-49, 0.8f, 2000, true);
                    } else if (glyphColumnKey==RelicRecoveryVuMark.CENTER) {
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                42,
                                0.75f,
                                5000,
                                true,
                                true);
                        drive.turn(-50, 0.6f, 2000, true);
                    } else {
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                34,
                                0.75f,
                                5000,
                                true,
                                true);
                        drive.turn(-101, 0.65f, 6000, true);
                    }
                    /**
                     *  red and near: move into position to deposit the first glyph
                     */

                } else {
                    if (glyphColumnKey==RelicRecoveryVuMark.RIGHT) {
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                -38,
                                0.75f,
                                5000,
                                true,
                                true);
                        drive.turn(-97, 0.7f, 3000, false);
                    } else if (glyphColumnKey==RelicRecoveryVuMark.CENTER) {
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                -32,
                                0.75f,
                                5000,
                                true,
                                true);
                        drive.turn(-44, 0.7f, 6000, false);
                    } else {
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                -40,
                                0.75f,
                                5000,
                                true,
                                true);
                        drive.turn(-44, 0.7f, 6000, false);
                    }
                }

                /**
                 * dump the glyph and push in
                 */

                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);

                lp.waitMillis(500);
                robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
                lp.waitMillis(200);
                drive.moveToTime(-0.45f, 600);
                drive.moveToTime(0.5f, 500);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                /**
                 *  if there is enough time, go for the second and third glyph
                 */

                long nearTime;
                nearTime = System.currentTimeMillis();

                telemetry.addData("time", nearTime-startTime);
                telemetry.update();
                if ( isGlyph&&nearTime-startTime<15000 /*(glyphColumnKey==RelicRecoveryVuMark.RIGHT ||
                        glyphColumnKey==RelicRecoveryVuMark.LEFT || glyphColumnKey==RelicRecoveryVuMark.CENTER)*/) {
                    //correction of heading so that we can go into the glyph pit straight
                    if ((glyphColumnKey==RelicRecoveryVuMark.RIGHT && isRed) ||
                            (glyphColumnKey==RelicRecoveryVuMark.LEFT && !isRed)) {
                        if (!isRed) {
                            drive.turn(-22,
                                    0.35f,
                                    2000,
                                    false);
                        } else {
                            drive.setTurnPower(0.4f);
                            lp.waitMillis(350);
                        }
                        RobotLog.i("im here");
                    } else if (glyphColumnKey==RelicRecoveryVuMark.CENTER){

                        drive.turn(-27,
                                0.5f,
                                3000,
                                false);

                    } else {
                        if (!isRed) {
                            RobotLog.v("here blue near after dump");
                            float headingN = robot.gyroSensor.getHeading();
                            while (headingN < -1 || headingN > 1) {
                                if (headingN > 1) {
                                    drive.setPower(0.5f, -0.5f);
                                } else {
                                    drive.setPower(-0.5f, 0.5f);
                                }
                                headingN = robot.gyroSensor.getHeading();
                                telemetry.addData("heading", headingN);
                                telemetry.update();
                            }
                            drive.turn(-8,
                                    0.5f,
                                    3000,
                                    false);
                        } else{
                            drive.turn(-27, 0.5f, 3000,
                                    false);

                        }

                    }
                    /**
                     *go to the glyph pit
                     */
                    float sonarCm2;
                    sonarCm2 = (robot.sonarRF.getDistance()-12.7f)*2+50;
                    if (sonarCm2 > 100) {
                        sonarCm2 = 100;
                    }
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
                                -8,
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
                        lp.waitMillis(100);
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                8,
                                0.7f,
                                600,
                                true,
                                true);
                        lp.waitMillis(400);
                        countTray = glyphCount.GetGlyphCount(0);
                        if (countTray !=2 ) {
                            drive.turn(-20,
                                    0.4f,
                                    2000,
                                    false);
                            countTray = glyphCount.GetGlyphCount(0);
                            if (countTray != 2) {
                                drive.moveToEncoderInch(TurnType.FORWARD,
                                        15,
                                        0.8f,
                                        600,
                                        true,
                                        true);
                                drive.moveToEncoderInch(TurnType.FORWARD,
                                        -15,
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
                                MoreGlyphs = true;
                            } else {
                                drive.turn(20,
                                        0.5f,
                                        2000,
                                        false);
                                robot.sweeperLeft.setPower(0);
                                robot.sweeperRight.setPower(0);
                                drive.moveToTime(-0.4f, 800);
                                robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                                MoreGlyphs=true;
                            }
                        }
                        else {
                            robot.sweeperLeft.setPower(0);
                            robot.sweeperRight.setPower(0);
                            drive.moveToTime(-0.4f, 800);
                            robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                            MoreGlyphs = true;
                        }
                    } else {
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                -8,
                                0.75f,
                                2000,
                                true,
                                true);
                        robot.sweeperLeft.setPower(0);
                        robot.sweeperRight.setPower(0);
                        drive.moveToTime(-0.4f, 800);
                        robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                        MoreGlyphs = false;
                    }
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
                    /**
                     *    go back to the cryptobox
                     */
                    if (MoreGlyphs) {
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                32 - Math.round(sonarCm2 / 2.54f),
                                0.8f,
                                3000,
                                true,
                                true
                        );
                    } else {
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                36 - Math.round(sonarCm2 / 2.54f),
                                0.8f,
                                3000,
                                true,
                                true);
                        RobotLog.aa("back driving distance", "%d", Math.round(sonarCm2/2.54));
                    }
                    robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_IN);
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLATA);
                    lp.waitMillis(200);
                    robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_OUT);
                    robot.sweeperLeft.setPower(-0.8f);
                    robot.sweeperRight.setPower(-0.8f);
                    if ((glyphColumnKey==RelicRecoveryVuMark.LEFT && !isRed) ||
                            (glyphColumnKey==RelicRecoveryVuMark.RIGHT&& isRed)) {
                        if (!isRed) {
                            drive.moveToTime(-0.45f, 375);
                            drive.turn(-25,
                                    0.4f,
                                    3000,
                                    false);
                        } else {
                            drive.moveToTime(-0.4f, 400);
                            drive.moveToEncoderInch(TurnType.STRAFE,
                                    -3,
                                    0.6f,
                                    2000,
                                    true,
                                    true);
                            drive.turn(14,
                                    0.5f,
                                    4000,
                                    false);
                        }
                        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                        drive.moveToTime(-0.35f, 400);
                        lp.waitMillis(200);
                        robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
                        drive.moveToTime(-0.5f,350);
                        drive.moveToTime(0.4f, 400);
                        lp.waitMillis(250);
                        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                        robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                        drive.moveToTime(-0.4f, 600);
                        drive.moveToTime(0.5f, 300);

                    } else if (glyphColumnKey==RelicRecoveryVuMark.CENTER) {
                        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLATA);
                        //robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_OUT);

                        if (!isRed) {
                            drive.moveToTime(-0.4f, 300);
                            drive.moveToEncoderInch(TurnType.STRAFE,
                                    4,
                                    0.8f,
                                    4000,
                                    true,
                                    false);
                        } else {
                            drive.moveToTime(-0.4f, 400);
                        }
                        drive.turn(isRed?MoreGlyphs?-25:-13:-17,
                                0.3f,
                                2000,
                                false);

                        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                        drive.moveToTime(-0.45f, 500);
                        lp.waitMillis(200);
                        robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
                        drive.moveToTime(-0.5f,350);
                        drive.moveToTime(0.4f, 400);
                        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                        robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                        drive.moveToTime(-0.4f, 800);
                        drive.moveToTime(0.3f, 500);
                    } else{
                        if (!isRed&&MoreGlyphs) {
                            drive.moveToTime(-0.45f, 375);
                            /*drive.moveToEncoderInch(TurnType.STRAFE,
                                    4,
                                    0.6f,
                                    2000,
                                    true,
                                    true);*/
                            drive.turn(19,
                                    0.5f,
                                    2000,
                                    false);
                        } else if (isRed){
                            drive.moveToTime(-0.4f, 450);
                            /*drive.moveToEncoderInch(TurnType.STRAFE,
                                    -7, 0.5f, 2000, false, true);*/
                            drive.turn(-31,
                                    0.4f,
                                    3000,
                                    false);
                        } else {
                          drive.setPower(-0.4f, 0.4f);
                          lp.waitMillis(100);
                        }
                        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
                        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                        drive.moveToTime(-0.45f, 600);
                        robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
                        lp.waitMillis(250);
                        drive.moveToTime(0.4f, 400);

                        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                        robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                        drive.moveToTime(-0.4f, 600);
                        drive.moveToTime(0.5f, 300);
                    }
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);

                }
            }

            robot.sweeperLeft.setPower(-0);
            robot.sweeperRight.setPower(-0);
            /**
             *            for blue or red and far
             */
            if (!isNear) {
                float headingFar = robot.gyroSensor.getHeading();
                int strafeFar = 15;
                lp.waitMillis(100);
                robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_IN);
                drive.moveToEncoderInch(TurnType.FORWARD,
                        isRed ? -24 : 24,
                        0.8f,
                        2500,
                        true,
                        false);
                /**
                 *drive and turn differently based on vumark pictograph
                 */

                if ((glyphColumnKey==RelicRecoveryVuMark.LEFT && !isRed )||
                        ( glyphColumnKey==RelicRecoveryVuMark.RIGHT && isRed)) {
                    drive.turn(isRed?14:144,
                            0.6f,
                            3000,
                            true);
                } else if (glyphColumnKey == RelicRecoveryVuMark.CENTER) {
                    drive.moveToEncoderInch(TurnType.STRAFE,
                            isRed?4:4,
                            0.6f,
                            1000,
                            true,
                            true);
                    drive.turn(isRed?24:135,
                            0.6f,
                            4000,
                            true);
                } else {
                    drive.moveToEncoderInch(TurnType.STRAFE,
                            isRed?13:13,
                            0.8f,
                            3000,
                            true, false);
                    RobotLog.vv("far strafing", "%d", strafeFar);
                    drive.turn(isRed?24: 130,
                            0.8f,
                            3000,
                            true
                    );
                }
                /**
                 *  dump the first glyph
                 */

                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                lp.waitMillis(300);
                robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
                drive.moveToTime(-0.45f, 400);
                lp.waitMillis(100);
                drive.moveToTime(0.4f, 300);
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                /**
                 *   for second and third glyphs
                 */
                long farTime;
                farTime= System.currentTimeMillis();
                if (isGlyph &&farTime-startTime<14000 && (glyphColumnKey==RelicRecoveryVuMark.RIGHT ||
                        glyphColumnKey==RelicRecoveryVuMark.LEFT || glyphColumnKey==RelicRecoveryVuMark.CENTER)) {
                robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_OUT);
                    if ((glyphColumnKey==RelicRecoveryVuMark.LEFT && !isRed) ||
                            (glyphColumnKey==RelicRecoveryVuMark.RIGHT && isRed)) {
                        drive.turn(isRed ? -8 : 24,
                                0.6f,
                                4000,
                                true);
                    } else if (glyphColumnKey==RelicRecoveryVuMark.CENTER){
                        drive.turn(isRed ? -24 : 30,
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
                            4,
                            0.75f,
                            1000,
                            true,
                            true);
                    /**
                     *   move into position to drive to the glyph pit using the range sensor
                     */
                    if (glyphColumnKey==RelicRecoveryVuMark.CENTER) {
                        drive.moveToEncoderInch(TurnType.STRAFE,
                                isRed?14:-16,
                                0.75f,
                                3000 ,
                                true,
                                false);
                    } else if ((glyphColumnKey==RelicRecoveryVuMark.RIGHT&&isRed) ||
                            (glyphColumnKey==RelicRecoveryVuMark.LEFT && !isRed)) {
                        drive.moveToEncoderInch(TurnType.STRAFE,
                                isRed? 16:-18,
                                0.65f,
                                3000,
                                true, false);
                    } else {
                        drive.moveToEncoderInch(TurnType.STRAFE,
                                isRed? 7:-5,
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
                        drive.turn(isRed? -22:36,
                                0.7f,
                                2000,
                                true);
                    }

                    robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_OUT);
                    robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                    robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                    drive.moveToEncoderInch(TurnType.FORWARD,
                            42,
                            1f,
                            4000,
                            true,
                            true);

                    lp.waitMillis(400);
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
                        lp.waitMillis(400);
                        countTrayFar = glyphCount.GetGlyphCount(0);
                        if (countTrayFar!=2) {
                            drive.moveToEncoderInch(TurnType.FORWARD,
                                    8,
                                    0.9f,
                                    700,
                                    true,
                                    false);
                            drive.moveToEncoderInch(TurnType.FORWARD,
                                    -8,
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
                                -26,
                                1f,
                                3000,
                                true,
                                true);
                    } else {
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                -32,
                                1f,
                                3000,
                                true,
                                true);
                    }
                    robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_IN);
                    if (glyphColumnKey==RelicRecoveryVuMark.CENTER){
                        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
                        robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);

                        if (isRed) {
                            if (!MoreGlyphs) {
                                drive.moveToTime(0.4f, 300);
                            } else {
                                drive.moveToTime(0.3f, 250);
                            }

                            drive.turn(MoreGlyphs?13:13, 0.4f, 4000, false);

                        }
                        if (!isRed) {
                            drive.moveToTime(0.35f,
                                    400);
                            drive.turn(8,
                                    0.5f,
                                    3000,
                                    false);
                        }
                        if (isRed||(!isRed && MoreGlyphs)) {
                            drive.moveToEncoderInch(TurnType.STRAFE,
                                    isRed ? MoreGlyphs?0:-11: -12,
                                    0.8f,
                                    3000,
                                    true,
                                    true);
                            if (isRed) {

                                drive.turn(7,
                                        0.6f,
                                        3000,
                                        false);
                            }
                        }
                        robot.slide.moveToEncoder(100);
                        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                        if (isRed) {

                            drive.moveToTime(-0.45f, 600);
                        } else {

                            drive.moveToTime(-0.45f, 800);
                        }
                        robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DOOR_OUT);

                        drive.moveToTime(0.4f, 600);

                        drive.turn(isRed?0:-12,
                                0.6f,
                                2000,
                                false);
                        robot.slide.moveToEncoder(75);

                        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                        robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);

                        drive.moveToEncoderInch(TurnType.STRAFE,
                                isRed?MoreGlyphs?-9:4:14,
                                0.8f,
                                2000,
                                true,
                                true);


                    } else if ((glyphColumnKey==RelicRecoveryVuMark.RIGHT && isRed) ||
                            (glyphColumnKey==RelicRecoveryVuMark.LEFT && !isRed)) {
                        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
                        robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);

                        if (isRed) {
                            drive.moveToTime(0.35f, 300);
                        } else {
                            drive.moveToEncoderInch(TurnType.STRAFE,
                                    MoreGlyphs? 5:2,
                                    0.5f,
                                    3000,
                                    true,
                                    true);
                        }
                        drive.turn(isRed? 9:6,
                                0.7f,
                                2000,
                                false);
                        drive.moveToEncoderInch(TurnType.FORWARD,
                                isRed?-5:0,
                                0.8f,
                                2000,
                                true,
                                true);
                        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP-10);
                        drive.moveToTime(-0.4f, 700);
                        robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
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


                    } else {

                        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
                        robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                        if (isRed) {
                            drive.turn(12,
                                    0.6f,
                                    3000,
                                    false);
                            drive.moveToTime(-0.2f, 300);
                        }
                        drive.setPower(0);

                        drive.moveToEncoderInch(TurnType.STRAFE,
                                isRed? -16:16,
                                0.7f,
                                2000,
                                false,
                                true);
                        if (!isRed) {
                            /*drive.turn(6,
                                    0.6f,
                                    3000,
                                    false);*/
                        }
                        //robot.slide.moveToEncoder(100);
                        if (!isRed) {

                            drive.moveToTime(-0.45f, 500);
                        }
                        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                        if (isRed) {
                            drive.moveToTime(-0.4f, 300);
                            robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DOOR_OUT);
                            drive.moveToTime(0.3f, 300);
                            drive.moveToTime(-0.4f, 300);

                        }
                        robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DOOR_OUT);
                        drive.moveToTime(-0.45f, 300);
                        lp.waitMillis(100);
                        drive.moveToTime(0.4f, 400);
                        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                        robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                        robot.slide.moveToEncoder(30);
                        drive.moveToEncoderInch(TurnType.STRAFE,
                                6,
                                0.7f,
                                3000,
                                true,
                                true);

                    }
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                }
            }
            long timeStamp = System.currentTimeMillis();


            RobotLog.i("AutonomousRevamped finish in " + (System.currentTimeMillis() - startTime) + " milliseconds");
            lp.waitMillis(1000);
        }
    }


