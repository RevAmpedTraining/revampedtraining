package org.firstinspires.ftc.teamcode.Test;

/**
 * Created for the Control Sheet
 */

import com.revAmped.components.Button;
import com.revAmped.components.HwSonarAnalog;
import com.revAmped.components.TurnType;
import com.revAmped.config.RobotRevAmpedConstants;
import com.revAmped.linear.components.MecanumDriveLinear;
import com.revAmped.linear.components.RobotRevAmpedLinear2;
import com.revAmped.linear.util.SelectLinear;
import com.revAmped.linear.util.WaitLinear;
import com.revAmped.sensors.RevColorDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
/**
 * Autonomous Program for Revamped 12808
 * This is meant for the control sheet and was made 2/21/2018
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutonomousRevamped2", group="Game")
public class AutoForControl
        extends LinearOpMode
{
    //Revamped robot class
    private RobotRevAmpedLinear2 robot = null;
    //Revamped mecanum drive class
    private MecanumDriveLinear drive;
    //ultrasonic sensor for detecting glyphs
    private HwSonarAnalog sensor_sonar_front;

    private final static int MIN_FRONT_SPACING = -1;

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
            telemetry.update();
            if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {
                break;
            }
        }
        SelectLinear sl = new SelectLinear(this);
        boolean isRed = sl.selectAlliance();
        boolean isNear = sl.selectPosition();
        //int waitMillis = sl.adjustDelay();


        telemetry.addData("Alliance", isRed ? "Red" : "Blue");
        telemetry.addData("Position", isNear ? "Near" : "Far");

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
        RelicRecoveryVuMark glyphColumnKey = robot.relicRecoveryVuMark.getGlyphColumnKey();
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
        lp.waitMillis(300);
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
                    //testing 2/20/2018
                    drive.moveToEncoderInch(TurnType.FORWARD,
                            48,
                            0.5f,
                            5000,
                            true,
                            true);
                    drive.turn(-50, 0.35f, 2000, false);
                } else if (glyphColumnKey==RelicRecoveryVuMark.CENTER) {
                    drive.moveToEncoderInch(TurnType.FORWARD,
                            58,
                            0.5f,
                            5000,
                            true,
                            true);
                    drive.turn(-50, 0.35f, 2000, false);
                } else if (glyphColumnKey==RelicRecoveryVuMark.RIGHT) {
                    drive.moveToEncoderInch(TurnType.FORWARD,
                            40,
                            0.5f,
                            5000,
                            true,
                            true);
                    drive.turn(-120, 0.35f, 6000, false);
                }
                /**
                 *  red and near: move into position to deposit the first glyph
                 */

            }
            lp.waitMillis(100);
            /**
             * dump the glyph and push in
             */
            drive.moveToTime(-0.3f, 500);
            robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DUMP);
            lp.waitMillis(1100);
            robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
            lp.waitMillis(100);
            drive.moveToTime(-0.3f, 800);
            drive.moveToTime(0.3f, 400);

            /**
             *  if there is enough time, go for the second and third glyph
             */

            long nearTime;
            nearTime = System.currentTimeMillis();
            telemetry.addData("time", nearTime-startTime);
            telemetry.update();
            if (nearTime-startTime<18000) {
                robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                lp.waitMillis(300);
                //correction of heading so that we can go into the glyph pit straight
                if (glyphColumnKey==RelicRecoveryVuMark.RIGHT ) {
                    drive.turn(24,
                            0.35f,
                            2000,
                            false);
                } else if (glyphColumnKey==RelicRecoveryVuMark.CENTER){
                    drive.turn(-30,
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
                sonarCm2 = (robot.sonarRF.getDistance()-12.7f)*2+60;
                telemetry.addData("sonar distance", sonarCm2);
                telemetry.update();
                robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_OUT);
                robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                float startEncoderValue = drive.getEncoder();
                drive.moveToEncoderInch(TurnType.FORWARD,
                        Math.round(sonarCm2/2.54f),
                        0.8f,
                        1500,
                        true,
                        true);
                lp.waitMillis(100);
                /**
                 *  go in twice to get two glyphs. also reverse intake briefly to spit out clogged glyphs
                 */
                drive.moveToEncoderInch(TurnType.FORWARD,
                        -10,
                        0.7f,
                        600,
                        true,
                        true);
                lp.waitMillis(300);
                robot.sweeperLeft.setPower(-0.6f);
                robot.sweeperRight.setPower(-0.6f);
                lp.waitMillis(300);
                drive.moveToEncoderInch(TurnType.FORWARD,
                        -10,
                        0.75f,
                        2000,
                        true,
                        true);
                robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                lp.waitMillis(1000);
                robot.sweeperLeft.setPower(0);
                robot.sweeperRight.setPower(0);
                /**
                 *    go back to the cryptobox
                 */
                drive.moveToEncoderInch(TurnType.FORWARD,
                        20-Math.round(sonarCm2/2.54f),
                        0.8f,
                        3000,
                        true,
                        true
                );
                if (glyphColumnKey==RelicRecoveryVuMark.LEFT) {
                    drive.turn(-25,
                            0.4f,
                            2000,
                            false);
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
                    drive.moveToTime(-0.3f, 500);
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                    lp.waitMillis(400);
                    drive.moveToTime(-0.3f, 300);
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                    drive.moveToTime(0.3f, 300);

                } else if (glyphColumnKey==RelicRecoveryVuMark.RIGHT) {
                        drive.turn(25,
                                0.5f,
                                2000,
                                false);
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
                    drive.moveToTime(-0.3f, 500);
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                    lp.waitMillis(400);
                    drive.moveToTime(-0.3f, 300);
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
                    drive.moveToTime(0.3f, 300);
                }

            }
        }

        /**
         *            for blue or red and far
         */
        if (!isNear) {
            int strafeFar = 15;
            lp.waitMillis(100);
            if (jewelcolor==RevColorDistanceSensor.COLORTYPE.BLUE) {
                drive.moveToEncoderInch(TurnType.FORWARD,
                        35,
                        0.6f,
                        2500,
                        true,
                        true);
            } else {
                drive.moveToEncoderInch(TurnType.FORWARD,
                        30,
                        0.6f,
                        2500,
                        true,
                        true);
            }
            telemetry.update();
            lp.waitMillis(200);
            /**
             *drive and turn differently based on vumark pictograph
             */

            if (glyphColumnKey==RelicRecoveryVuMark.LEFT) {
                drive.turn(150,
                        0.4f,
                        3000,
                        false);
            } else if (glyphColumnKey == RelicRecoveryVuMark.CENTER) {
                drive.moveToEncoderInch(TurnType.STRAFE,
                        3,
                        0.5f,
                        1000,
                        true,
                        true);
                drive.turn(142,
                        0.3f,
                        3000,
                        false);
            } else {
                drive.moveToEncoderInch(TurnType.STRAFE,
                        16,
                        0.5f,
                        3000,
                        true, true);
                drive.turn(140,
                        0.35f,
                        3000,
                        false
                );
            }
            /**
             *  dump the first glyph
             */
            drive.moveToTime(-0.35f, 350);
            lp.waitMillis(250);
            robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DUMP);
            lp.waitMillis(1500);
            robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
            drive.moveToTime(0.35f, 300);
            lp.waitMillis(100);
            drive.moveToTime(-0.35f, 600);
            drive.moveToTime(0.35f, 300);
            robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
            /**
             *   for second and third glyphs
             */
            long farTime;
            farTime= System.currentTimeMillis();
            if (farTime-startTime<14000) {
                lp.waitMillis(150);
                if (glyphColumnKey==RelicRecoveryVuMark.LEFT) {
                    drive.turn(20,
                            0.4f,
                            2000,
                            false);
                } else {
                    drive.turn(24,
                            0.4f,
                            2000,
                            false);
                }
                drive.moveToEncoderInch(TurnType.FORWARD,
                        6,
                        0.75f,
                        1000,
                        true,
                        true);
                /**
                 *   move into position to drive to the glyph pit using the range sensor
                 */
                if (glyphColumnKey==RelicRecoveryVuMark.CENTER) {
                    drive.moveToEncoderInch(TurnType.STRAFE,
                            -22,
                            0.65f,
                            3000 ,
                            true,
                            true);
                } else if (glyphColumnKey==RelicRecoveryVuMark.LEFT) {
                    drive.moveToEncoderInch(TurnType.STRAFE,
                            -30,
                            0.65f,
                            3000,
                            true, true);
                } else {
                    drive.moveToEncoderInch(TurnType.STRAFE,
                            -6,
                            0.65f,
                            3000,
                            true,
                            true);
                }
                if (glyphColumnKey==RelicRecoveryVuMark.LEFT ) {
                    drive.turn(25,
                            0.4f,
                            2000,
                            false);
                } else  {
                    drive.turn(32,
                            0.5f,
                            2000,
                            false);
                }

                drive.moveToEncoderInch(TurnType.FORWARD,
                        45,
                        0.8f,
                        3000,
                        true,
                        true);
                robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_OUT);
                lp.waitMillis(100);
                /**
                 *         intake two glyphs
                 */
                drive.moveToEncoderInch(TurnType.FORWARD,
                        20,
                        0.7f,
                        600,
                        true,
                        true);
                lp.waitMillis(100);
                drive.moveToEncoderInch(TurnType.FORWARD,
                        -20,
                        0.7f,
                        600,
                        true,
                        true);
                lp.waitMillis(100);
                robot.sweeperLeft.setPower(-0.6f);
                robot.sweeperRight.setPower(-0.6f);
                lp.waitMillis(300);
                robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
                drive.moveToEncoderInch(TurnType.FORWARD,
                        -10,
                        0.8f,
                        1000,
                        true,
                        true);

                robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
                lp.waitMillis(1000);
                robot.sweeperLeft.setPower(0);
                robot.sweeperRight.setPower(0);
                drive.moveToEncoderInch(TurnType.FORWARD,
                        -32,
                        0.8f,
                        2000,
                        true,
                        true);
                lp.waitMillis(200);
                if (glyphColumnKey==RelicRecoveryVuMark.RIGHT) {
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
                    drive.moveToEncoderInch(TurnType.FORWARD,
                            -7,
                            0.8f,
                            2000,
                            true,
                            true);
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                    lp.waitMillis(400);
                    drive.moveToTime(-0.3f, 300);
                    drive.moveToTime(0.3f, 300);

                }else if (glyphColumnKey==RelicRecoveryVuMark.LEFT) {
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
                    drive.moveToEncoderInch(TurnType.FORWARD,
                            -11,
                            0.8f,
                            2000,
                            true,
                            true);
                    drive.turn(-20,
                            0.4f,
                            2000,
                            false);
                    robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
                    lp.waitMillis(400);
                    drive.moveToTime(-0.3f, 300);
                    drive.moveToTime(0.3f, 300);
                } else {
                    drive.turn(-6,
                            0.4f,
                            3000,
                            false);

                    drive.moveToEncoderInch(TurnType.STRAFE,
                             8,
                            0.4f,
                            5000,
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



