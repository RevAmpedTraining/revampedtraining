package org.firstinspires.ftc.teamcode.Test;

import com.revAmped.components.TurnType;
import com.revAmped.config.RobotOverchargedConstants;
import com.revAmped.linear.components.RobotOverchargedLinear2;
import com.revAmped.linear.components.SwerveDriveLinear;
import com.revAmped.linear.util.SelectLinear;
import com.revAmped.linear.util.WaitLinear;
import com.revAmped.sensors.RevColorDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static com.revAmped.config.RobotOverchargedConstants.SERVO_CONTAINER_NORMAL;
import static com.revAmped.config.RobotOverchargedConstants.SERVO_CONTAINER_SCORE;
import static com.revAmped.config.RobotOverchargedConstants.SERVO_JEWEL_KOCKER_LEFT;
import static com.revAmped.config.RobotOverchargedConstants.SERVO_JEWEL_KOCKER_MID;
import static com.revAmped.config.RobotOverchargedConstants.SERVO_JEWEL_KOCKER_RIGHT;
import static com.revAmped.config.RobotOverchargedConstants.TAG_A;

/**
 * Overcharged Team #12599 Autonomous
 * This program is divided into 3 main common functions
 * Each one of them is used in all 4 positions Blue Near, Blue Far, Red Near & Red Far
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "OverchargedTempTest", group = "Game")
public class OverchargedTempTest
        extends LinearOpMode {
    ///Overcharged Autonomous Robot class
    private RobotOverchargedLinear2 robot;
    ///Overcharged Swirve Drive class
    private SwerveDriveLinear drive;
    ///The Jewel color sensor used
    private RevColorDistanceSensor sensorColor;
    ///Ultrasonic sensors used to in the front of the robot
    private ModernRoboticsI2cRangeSensor rangeSensorFront;
    ///Ultrasonic sensors used at the back of the robot
    private ModernRoboticsI2cRangeSensor rangeSensorBack;
    /**
     * Time to wait for the Jewel servo to perform the indicated action
     * Bring the arm in
     */
    private final static int JEWEL_IN_WAIT_TIME = 800;
    /**
     * Preset common distance to get of the board for all 4 positions
     */
    private final static int OFF_BOARD_DISTANCE = 23;
    /**
     * Time to wait for the Jewel servo to perform the indicated action
     * Bring the arm out
     */
    private final static int JEWEL_OUT_WAIT_TIME = 200;
    /**
     * Time to wait for the slide to perform the indicated action
     * Bring the slide up/down
     */
    private final static int SLIDE_WAIT_TIME = 200;
    /**
     * Time to wait for the Dumper servo to perform the indicated action
     * Bring the dumper to TILT/SCORE/NORMAL positions
     */
    private final static int DUMPER_WAIT_TIME = 200;
    /**
     * Time to wait for the Intake to perform the indicated action
     * Bring the motors to REJECT/INTAKE/STOP positions
     */
    private final static int INTAKE_WAIT_TIME = 300;
    /**
     * autonomous opMode, entry point to the program
     */
    @Override
    public void runOpMode()
            throws InterruptedException {
        try {
            // init
            robot = new RobotOverchargedLinear2(this);
            RobotLog.ii(TAG_A, "RobotOverchargedLinear initialized");
            drive = robot.getSwerveDriveLinear();
            RobotLog.ii(TAG_A, "SwerveDriveLinear initialized");
            run();
        } finally {
            // shut down
            if (robot != null) {
                robot.close();
            }
        }
    }

    /**
     * Knock the Jewel off and get down from the balance board
     * @param lp a WaitLinear type to handle delays/waits
     * @param isJewelBlue a Boolean object indicating if the Jewel color is Blue. True for Blue, False for Red and null for could not read
     * @param isNear a boolean value indicating if position is Near or Far.
     * @param isRed a boolean value indicating if its a Red or Blue alliance.
     */
    private void knockOffJewelAndGetOffTheBoard(WaitLinear lp, Boolean isJewelBlue, boolean isNear, boolean isRed)
            throws InterruptedException {
        RobotLog.ii(TAG_A, "knockOffJewelAndGetOffTheBoard: isRed=" + isRed + " isNear=" + isNear + " isJewelBlue=" + ((isJewelBlue != null) ? isJewelBlue.booleanValue() : "null"));

        if (isJewelBlue != null) {
            //Was able to read the Jewel so knock it off
            if (isJewelBlue.booleanValue() == Boolean.TRUE) {
                robot.servoJewelKnocker.setPosition(isRed ? SERVO_JEWEL_KOCKER_RIGHT : SERVO_JEWEL_KOCKER_LEFT);
            } else {
                robot.servoJewelKnocker.setPosition(isRed ? SERVO_JEWEL_KOCKER_LEFT : SERVO_JEWEL_KOCKER_RIGHT);
            }
        }

        lp.waitMillis(JEWEL_IN_WAIT_TIME);
        robot.servoJewel.setPosition(RobotOverchargedConstants.SERVO_JEWEL_IN);
        robot.servoJewelKnocker.setPosition(SERVO_JEWEL_KOCKER_MID);
        lp.waitMillis(JEWEL_IN_WAIT_TIME);

        drive.moveToEncoderInch(TurnType.FORWARD,
                isRed ? -OFF_BOARD_DISTANCE : OFF_BOARD_DISTANCE,
                0.2f,
                30000,
                false,
                false,
                true);
    }

    /**
     * This action moves the robot back and pushes the glyph in to realign the column if necessary
     */
    private void realignTheGlyphs() throws InterruptedException {
        //Move out and push again
        drive.moveToEncoderInch(TurnType.FORWARD,
                2,
                0.2f,
                30000,
                false,
                false,
                true);
        //push again
        drive.moveToEncoderInch(TurnType.FORWARD,
                -4,
                0.2f,
                30000,
                false,
                false,
                true);
    }

    /**
     * This function moves to the required distance based on the key column read by Vuforia, scores the Glyph in the key column and
     * attempts to collect more from the Glyph pit and score into the key column
     * @param glyphColumnKey an RelicRecoveryVuMark argument to indicate the key column to move to, LEFT, CENTER or RIGHT. Null indicates CENTER
     * @param isRed a boolean value indicating if its a Red or Blue alliance.
     * @param isNear a boolean value indicating if position is Near or Far.
     * @param isJewelBlue a Boolean object indicating if the Jewel color is Blue. True for Blue, False for Red and null for could not read
     * @param lp a WaitLinear type to handle delays/waits
     * @param collectAdditional a boolean value indicating if the driver coach selected Yes to attempt additional glyphs
     * @param scoreAdditional a boolean value indicating if the driver coach selected Yes to score additional glyphs
     * @param startTime a long value indicating the start time when the start was hit for this autonomous program
     */
    private void findKeyColumnAndScore(RelicRecoveryVuMark glyphColumnKey, boolean isRed, boolean isNear, Boolean isJewelBlue, WaitLinear lp, boolean collectAdditional, boolean scoreAdditional, long startTime)
            throws InterruptedException {
        RobotLog.ii(TAG_A, "findKeyColumnAndScore: isRed=" + isRed + " isNear=" + isNear + " glyphColumnKey=" + glyphColumnKey.toString());
        int strafeDistance = getStrafeDistance(glyphColumnKey, isRed, isNear, isJewelBlue);
        int turnAngle = isNear ? -83 : (isRed ? 3 : 174);
        if (turnAngle != 0) {
            drive.turn(TurnType.TURN_REGULAR,
                    turnAngle,
                    0.20f, -1,
                    true);
        }

        RobotLog.ii(TAG_A, "findKeyColumnAndScore: Strafe=" + strafeDistance + " turnAngle=" + turnAngle);

        drive.moveToEncoderInch(TurnType.STRAFE,
                isRed ? strafeDistance : -strafeDistance,
                0.2f,
                30000,
                false,
                false,
                true);

        ///Record the Heading so that it can be used later at Check point
        float startHeading = robot.gyroSensor.getHeading();

        //Move into the cryptobox
        robot.servoDumper.setPosition(SERVO_CONTAINER_SCORE);
        drive.moveToTime(TurnType.FORWARD, -0.25f,
                600);
        lp.waitMillis(DUMPER_WAIT_TIME);

        ///Realign the column as necessary
        realignTheGlyphs();
        int backoutDistance = getBackoutDistance(glyphColumnKey, isNear);
        //back out
        drive.moveToEncoderInch(TurnType.FORWARD,
                backoutDistance,
                0.3f,
                30000,
                false,
                false,
                true);
        //Move slide down
        robot.slide.moveToBottom();
        lp.waitMillis(SLIDE_WAIT_TIME);
        robot.servoDumper.setPosition(SERVO_CONTAINER_NORMAL);

        long currentTime = System.currentTimeMillis();
        RobotLog.ii(TAG_A, "findKeyColumnAndScore: Time Remaining=" + (currentTime-startTime));
        //Do we have enough time to collect the second glyph
        //if (currentTime-startTime < 20000) {
        //	isCollectAdditional = false;
        //}
        if (collectAdditional && isNear) {
            //Red or Blue and Near
            //scoreAdditionalGlyps(glyphColumnKey, isRed, isNear, isJewelBlue, lp, startHeading, scoreAdditional, startTime, backoutDistance);
        }
    }


    /**
     * Get the distance in inches to travel based on sonar sensor
     * 28.23 == 22
     * 29.16 == 21 inches travel
     * 31.65 == 24
     * 35.37 == 26
     * 40.64 == 30 inches travel
     * 49.02 = 35 inches
     */
    private int getDistanceToTravelBasedOnSonar() {
        float sonarValue = robot.sonarLF.getDistance();
        telemetry.addData("sonarValue:", sonarValue);
        if (sonarValue > 0 && sonarValue < 60) {
            return (int) ((sonarValue/38f) * 33);
        }
        return 26;
    }

    /**
     * This function attempts to collect more from the Glyph pit and score into the key column
     * @param glyphColumnKey an RelicRecoveryVuMark argument to indicate the key column to move to, LEFT, CENTER or RIGHT. Null indicates CENTER
     * @param isRed a boolean value indicating if its a Red or Blue alliance
     * @param isNear a boolean value indicating if position is Near or Far
     * @param isJewelBlue a Boolean object indicating if the Jewel color is Blue. True for Blue, False for Red and null for could not read
     * @param lp a WaitLinear type to handle delays/waits
     * @param startHeading a float value that contains the starting heading
     * @param scoreAdditional a boolean value indicating if the driver coach selected Yes to score additional glyphs
     * @param startTime a long value indicating the start time when the start was hit for this autonomous program
     * @param backoutDistance a int value indicating the distance to back out after scoring based on the key column
     */
    private void scoreAdditionalGlyps(RelicRecoveryVuMark glyphColumnKey, boolean isRed, boolean isNear, Boolean isJewelBlue, WaitLinear lp, float startHeading, boolean scoreAdditional, long startTime, int backoutDistance)
            throws InterruptedException {
        ///Check point #1: TO ensure that the robot is heading in the expected direction
        float currentHeading = robot.gyroSensor.getHeading();
        if (currentHeading != startHeading) {
            drive.turn(
                    TurnType.TURN_REGULAR,
                    startHeading - currentHeading,
                    0.20f, -1,
                    true
            );
        }
        ///Read the distance using sonar to figure out how much to travel
        int distance = getDistanceToTravelBasedOnSonar();
        telemetry.addData("Additional:", "startHeading=" + startHeading + " distance=" + distance);
        telemetry.update();
        lp.waitMillis(20000);
        /**
        ///Attempt to get a couple more Glyphs from the pit after depositing the initial Glyp into the key column
        robot.intake.collect();
        drive.moveToEncoderInch(TurnType.FORWARD,
                distance,
                0.5f,
                30000,
                false,
                false,
                true);
        telemetry.update();
        lp.waitMillis(5000);
        ///Attempt to collect the second Glyph
        drive.moveToEncoderInch(TurnType.FORWARD, -4,
                0.25f,
                30000,
                false,
                false,
                true);
        ///Reposition the collected Glyph
        robot.intake.reject();
        lp.waitMillis(300);
        ///Attempt to collect the third Glyph
        robot.intake.collect();
        drive.moveToEncoderInch(TurnType.FORWARD,
                4,
                0.25f,
                30000,
                false,
                false,
                true);
        robot.intake.collect();
        ///Check point #2: TO ensure that the robot is heading in the expected direction
        currentHeading = robot.gyroSensor.getHeading();
        if (currentHeading != startHeading) {
            drive.turn(
                    TurnType.TURN_REGULAR,
                    startHeading - currentHeading,
                    0.25f,
                    -1,
                    true
            );
        }
        robot.servoDumper.setPosition(SERVO_CONTAINER_TILT);
        lp.waitMillis(DUMPER_WAIT_TIME);
        robot.intake.stop();
        ///If the dumper is not in the correct position don't score
        telemetry.addData("Dumper:", "Position=" + robot.servoDumper.getPosition() + " SERVO_CONTAINER_TILT=" + SERVO_CONTAINER_TILT);
        //if (scoreAdditional && (robot.servoDumper.getPosition() <= SERVO_CONTAINER_TILT - 5 || robot.servoDumper.getPosition() >= SERVO_CONTAINER_TILT + 5)) scoreAdditional = false;

        drive.moveToEncoderInch(TurnType.FORWARD, -distance,
                0.5f,
                30000,
                false,
                false,
                true);
        lp.waitMillis(500);

        if (scoreAdditional) {
            //Don't want to do this in front of the key column that might mess up the already scored glyph
            //Do this before you reach the column
            robot.slide.moveToEncoder(RobotOverchargedConstants.SLIDE_LEVEL_TWO);
            lp.waitMillis(SLIDE_WAIT_TIME);
            //move slowly towards the column
            drive.moveToEncoderInch(TurnType.FORWARD, -backoutDistance,
                    0.2f,
                    30000,
                    false,
                    false,
                    true);
            lp.waitMillis(500);
            robot.servoDumper.setPosition(SERVO_CONTAINER_SCORE);
            lp.waitMillis(DUMPER_WAIT_TIME);
            ///Realign the column as necessary
            realignTheGlyphs();
        }

        robot.servoDumper.setPosition(SERVO_CONTAINER_TILT);
        lp.waitMillis(DUMPER_WAIT_TIME);
        robot.slide.moveToBottom();
        robot.servoDumper.setPosition(SERVO_CONTAINER_NORMAL);
         **/
    }

    /**
     * Get the column based on the Pictograph
     * @param glyphColumnKey an RelicRecoveryVuMark argument to indicate the key column to move to, LEFT, CENTER or RIGHT. Null indicates CENTER
     * @param forward a boolean value indicating if the robot is moving forward or backwards.
     */
    private int getColumn(RelicRecoveryVuMark glyphColumnKey, boolean forward) {
        int column = 2;
        if (!forward) {
            if (glyphColumnKey == RelicRecoveryVuMark.LEFT) {
                column = 3;
            } else if (glyphColumnKey == RelicRecoveryVuMark.RIGHT) {
                column = 1;
            } else {
                column = 2;
            }
        } else {
            if (glyphColumnKey == RelicRecoveryVuMark.LEFT) {
                column = 2;
            } else if (glyphColumnKey == RelicRecoveryVuMark.RIGHT) {
                column = 4;
            } else {
                column = 3;
            }
        }
        return column;
    }

    /**
     * Distance to Strafe for near and far.
     *!Total crypto box size is 24 inches.
     * So each column layout is 1.5 + 6 + 1.5 + 6 + 1.5 + 6 + 1.5 = 24.
     * 1.5 inches is the width of the border plastic.
     * 6 inches is the column inside width.
     * @param glyphColumnKey an RelicRecoveryVuMark argument to indicate the key column to move to, LEFT, CENTER or RIGHT. Null indicates CENTER
     * @param isRed a boolean value indicating if its a Red or Blue alliance.
     * @param isNear a boolean value indicating if position is Near or Far.
     * @param isJewelBlue a Boolean object indicating if the Jewel color is Blue. True for Blue, False for Red and null for could not read
     */
    //TODO: Make default glyphColumnKey as CENTER, Offboard distance 23
    private int getStrafeDistance(RelicRecoveryVuMark glyphColumnKey, boolean isRed, boolean isNear, Boolean isJewelBlue)
            throws InterruptedException {

        int strafeDistance = 4;

        if (glyphColumnKey == RelicRecoveryVuMark.LEFT) {
            if (isRed) {
                //Red alliance and far
                //This column is farthest (far from the robot)
                //4.5 + 7.5 + 7.5
                strafeDistance = isNear ? 18 : 19;
            } else {
                //Blue alliance and far
                //This column is nearest (near to the robot)
                strafeDistance = 4;
            }
        } else if (glyphColumnKey == RelicRecoveryVuMark.RIGHT) {
            if (isRed) {
                //Red alliance and far
                //This column is nearest (near to the robot)
                strafeDistance = 4;
            } else {
                //Blue alliance and far
                //This column is farthest (far from the robot)
                strafeDistance = 19;
            }
        } else {
            //RelicRecoveryVuMark.CENTER
            //For CENTER and UNKNOWN do the center so that we can get it in somewhere
            //4.5 + 7.5
            strafeDistance = 11;
        }
        return strafeDistance;
    }

    /**
     * Position the Robot at the Key column based on Range sensors
     * @param startEncodeValue a int value to indicate the starting encoder value.
     * @param maxDistanceAllowed a int value indicating the max distance to travel before giving up.
     * @param rangeSensor a ModernRoboticsI2cRangeSensor value indicating which range sensor to use Front/Back.
     * @param wallDistance a boolean value indicating if position is Near or Far.
     * @param power a float value indicating the drive power to use when moving.
     * @param stopAtColumn a int value indicating which column to stop at.
     */
    private boolean positionAtColumn(int startEncodeValue, int maxDistanceAllowed, ModernRoboticsI2cRangeSensor rangeSensor, double wallDistance, float power, int stopAtColumn) {
        double currentWallDistance = 0;
        int col = 0;
        int currentEncoderValue = 0;
        boolean sawColumnPost = false;
        boolean foundColumn = false;
        double distanceoffset = 0;
        while ((currentEncoderValue - startEncodeValue) < maxDistanceAllowed) {
            telemetry.addData("", "currentEncoderValue=" + currentEncoderValue + " startEncodeValue=" + startEncodeValue);
            currentWallDistance = rangeSensor.getDistance(DistanceUnit.CM);
            distanceoffset = wallDistance - currentWallDistance;
            if (distanceoffset > 4) {
                if (!sawColumnPost) {
                    //first time find needs to be processed
                    col++;
                    sawColumnPost = true;
                }
            } else {
                sawColumnPost = false;

            }
            if (col == stopAtColumn) {
                foundColumn = true;
                telemetry.addData("Key:", "foundColumn");
                break;
            }
            if (col > stopAtColumn) {
                break;
            }
            drive.setPower(power);
            currentEncoderValue = drive.getEncoder(TurnType.FORWARD);
        }
        return foundColumn;
    }

    /**
     * Get the distance to backout distance. This varies based on the position
     * @param glyphColumnKey an RelicRecoveryVuMark argument to indicate the key column to move to, LEFT, CENTER or RIGHT. Null indicates CENTER
     * @param isNear a boolean value indicating if position is Near or Far.
     */
    private int getBackoutDistance(RelicRecoveryVuMark glyphColumnKey, boolean isNear)
            throws InterruptedException {
        //don't back out more for left and right column
        int backoutDistance = isNear ? 4 : 5;
        if (glyphColumnKey == RelicRecoveryVuMark.CENTER) {
            backoutDistance = 6;
        }
        return backoutDistance;
    }

    /**
     * Autonomous run function
     * @throws InterruptedException
     */
    public void run()
            throws InterruptedException {

        sensorColor = new RevColorDistanceSensor(hardwareMap, "jewel_color");
        RobotLog.e("initialized jewel_color");
        rangeSensorFront = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensorFront");
        RobotLog.e("initialized rangeSensorFront");
        rangeSensorBack = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensorBack");
        RobotLog.e("initialized rangeSensorBack");


        SelectLinear sl = new SelectLinear(this);
        ///Make the alliance selection (Red/Blue)
        boolean isRed = false;
        ///Make the position selection (Near/Far)
        boolean isNear = true;
        boolean collectAdditional = true;
        boolean scoreAdditional = false;

        //int waitMillis = sl.adjustDelay();
        telemetry.addData("Alliance", isRed ? "Red" : "Blue");
        telemetry.addData("Position", isNear ? "Near" : "Far");

        //telemetry.addData("Delay", Integer.toString(waitMillis/1000));
        telemetry.addData("Waiting", "AutonomousOvercharged");
        telemetry.update();

        ///Set the LED indicating Blue or Red alliance  selected
        if (isRed) {
            robot.ledRed.on();
        } else {
            robot.ledBlue.on();
        }
        WaitLinear lp = new WaitLinear(this);

        robot.ledGreen.set(isNear);
        robot.drawLed();
        /// Reset the slide, move the slide to bottom until it touches the bottom switch and also set encoder to 0
        robot.slide.moveToBottom();
        ///Reset the Gyro sensor
        robot.gyroSensor.resetHeading();
        ///Bring in the Jewel Knocker inside the Robot
        robot.servoJewelKnocker.setPosition(RobotOverchargedConstants.SERVO_JEWEL_KOCKER_MID);
        ///Wait for Start to be pressed
        waitForStart();

        long startTime = System.currentTimeMillis();

        ///Turn Off all LEDs
        robot.ledRed.off();
        robot.ledBlue.off();
        robot.ledGreen.off();
        robot.ledWhite.off();
        robot.ledYellow.off();
        robot.drawLed();

        ///Read the Pictograph and get the Key column (LEFT, CENTER, RIGHT)
        RelicRecoveryVuMark glyphColumnKey = RelicRecoveryVuMark.CENTER;
        telemetry.addData("GlyphColumnKey:", "%s", glyphColumnKey);

        /** Read the Jewel color
         * 	Start from the bottom and keep moving up a bit until the Jewel color is read.
         *   If not read then deposit the Glyph in the center column
         */
        Boolean isJewelBlue = Boolean.TRUE;
        telemetry.addData("Jewel:", isJewelBlue == null ? "Null" : isJewelBlue.booleanValue() ? "blue" : "red");

        ///Call knockOffJewelAndGetOffTheBoard
        //knockOffJewelAndGetOffTheBoard(lp, isJewelBlue, isNear, isRed);
        ///Call findKeyColumnAndScore
        findKeyColumnAndScore(glyphColumnKey, isRed, isNear, isJewelBlue, lp, collectAdditional, scoreAdditional, startTime);
        telemetry.update();
        RobotLog.i("AutonomousOvercharged finish in " + (System.currentTimeMillis() - startTime) + " milliseconds");
        lp.waitMillis(5000);
        ///End
    }
}