package org.firstinspires.ftc.teamcode.OldStuff;

import com.revAmped.components.TurnType;
import com.revAmped.config.RobotOverchargedConstants;
import com.revAmped.linear.components.RobotOverchargedLinear;
import com.revAmped.linear.components.SwerveDriveLinear;
import com.revAmped.linear.util.SelectLinear;
import com.revAmped.linear.util.WaitLinear;
import com.revAmped.sensors.RevColorDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * AutonomousOvercharged
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutonomousOvercharged", group="Game")
public class AutonomousOvercharged
        extends LinearOpMode
{
    private RobotOverchargedLinear robot = null;

    private SwerveDriveLinear drive;
    private RevColorDistanceSensor sensorColor;

    private final static int JEWEL_IN_WAIT_TIME = 800;
    private final static int JEWEL_DISTANCE = 5;
    private final static int JEWEL_OUT_WAIT_TIME = 200;
    private final static int CLAW_WAIT_TIME = 500;

    /**
     * autonomous opMode
     */
    @Override
    public void runOpMode()
            throws InterruptedException {
        try {
            // init
            robot = new RobotOverchargedLinear(this);
            drive = robot.getSwerveDriveLinear();

            run();
        } finally {
            // shut down
            if (robot != null) {
                robot.close();
            }
        }
    }

    // Knock the Jewel off by turning
    private void knockOffJewelWithTurn(WaitLinear lp, boolean backwards)
            throws InterruptedException
    {
        int KnockJewelAngle = 10;
        //telemetry.addData("Jewel:", "knockOffJewelWithMove " + (backwards ? "backwards" : "forward"));
        drive.turn(TurnType.TURN_REGULAR,
                backwards ? KnockJewelAngle : -KnockJewelAngle,
                0.20f,
                -1,
                true);
        robot.servoJewel.setPosition(RobotOverchargedConstants.SERVO_JEWEL_IN);
        lp.waitMillis(JEWEL_IN_WAIT_TIME);
        drive.turn(TurnType.TURN_REGULAR,
                backwards ? -KnockJewelAngle : KnockJewelAngle,
                0.20f,
                -1,
                true);
        lp.waitMillis(250);
    }

    // Knock the Jewel off by moving forward
    private void knockOffJewelWithMove(WaitLinear lp, boolean forward)
            throws InterruptedException
    {
        //telemetry.addData("Jewel:", "knockOffJewelWithMove " + (forward ? "forward" : "backwards"));
        drive.moveToEncoderInch(TurnType.FORWARD,
                forward? JEWEL_DISTANCE : -JEWEL_DISTANCE,
                0.5f,
                30000,
                false,
                false,
                true);
        robot.servoJewel.setPosition(RobotOverchargedConstants.SERVO_JEWEL_IN);
        lp.waitMillis(JEWEL_IN_WAIT_TIME);
    }


    // Knock the Jewel off by moving forward
    private void dontKnockOffJewel(WaitLinear lp)
            throws InterruptedException
    {
        //telemetry.addData("Jewel:", "dontKnockOffJewel");
        robot.servoJewel.setPosition(RobotOverchargedConstants.SERVO_JEWEL_IN);
        lp.waitMillis(JEWEL_IN_WAIT_TIME);
    }


    //backout distnace for far only
    private int getBackoutDistance(RelicRecoveryVuMark glyphColumnKey, boolean isRed, boolean isNear, Boolean isJewelBlue)
            throws InterruptedException
    {
        //don't back out more for left and right column
        int backoutDistance = 3;
        if (isNear) {
            backoutDistance = 6;
        } else {
            if (glyphColumnKey == RelicRecoveryVuMark.CENTER) {
                backoutDistance = 6;
            }
        }
        return backoutDistance;
    }
    private int getSecondBackoutDistance(RelicRecoveryVuMark glyphColumnKey, boolean isRed, boolean isNear, Boolean isJewelBlue)
            throws InterruptedException
    {
        //don't back out more for left and right column
        int backoutDistance = 4;
        if (glyphColumnKey == RelicRecoveryVuMark.CENTER) {
            backoutDistance = 5;
        }
        return backoutDistance;
    }

    //Strafe distance for far only
    private int getFarStrafeDistance(RelicRecoveryVuMark glyphColumnKey, boolean isRed, boolean isNear, Boolean isJewelBlue)
            throws InterruptedException
    {
        if (isNear)
        {
            //This should never be called for isNear==true
            //This is only for far end so return 0 if near
            return 0;
        }

        int strafeDistance = 3;

        if (glyphColumnKey == RelicRecoveryVuMark.LEFT)
        {
            if (isRed)
            {
                //Red alliance and far
                //This column is farthest (far from the robot)
                if (isJewelBlue == null)
                {
                    //TODO not tested
                    strafeDistance = strafeDistance + 15;
                }
                else if (isJewelBlue.booleanValue())
                {
                    //Left Column tested good
                    //Blue Jewel, This is where we turn to knock off jewel
                    strafeDistance = strafeDistance + 15;
                }
                else
                {
                    //Left Column tested good
                    //Red Jewel, This is where we go straight to knock off jewel
                    strafeDistance = strafeDistance + 15;//Changed from 9 (which is center)
                }
            } else {
                //Blue alliance and far
                //This column is nearest (near to the robot)
                if (isJewelBlue == null) {
                    //TODO not tested
                    strafeDistance = strafeDistance + 1;
                }   else if (isJewelBlue.booleanValue()) {
                    //TODO not tested
                    //Blue Jewel, This is where we turn to knock off jewel
                    //strafeDistance = strafeDistance + 1;
                }   else {
                    //TODO not tested
                    //Red Jewel, This is where we go straight to knock off jewel
                    strafeDistance = strafeDistance + 1;
                }
            }
        }
        else if (glyphColumnKey == RelicRecoveryVuMark.RIGHT)
        {
            if (isRed)
            {
                //Red alliance and far
                //This column is nearest (near to the robot)
                if (isJewelBlue == null) {
                    //TODO not tested
                    strafeDistance = strafeDistance + 3;
                }   else if (isJewelBlue.booleanValue()) {
                    //TODO not tested
                    //Blue Jewel, This is where we turn to knock off jewel
                    strafeDistance = strafeDistance + 3;
                }   else {
                    //TODO not tested
                    //Red Jewel, This is where we go straight to knock off jewel
                    strafeDistance = strafeDistance + 3;
                }
            }
            else
            {
                //Blue alliance and far
                //This column is farthest (far from the robot)
                if (isJewelBlue == null)
                {
                    //TODO not tested
                    strafeDistance = strafeDistance + 13;
                }
                else if (isJewelBlue.booleanValue())
                {
                    //TODO not tested
                    //Blue Jewel, This is where we turn to knock off jewel
                    strafeDistance = strafeDistance + 12;
                }
                else
                {
                    //TODO not tested
                    //Red Jewel, This is where we go straight to knock off jewel
                    strafeDistance = strafeDistance + 13;
                }
            }
        }
        else
        {
            //RelicRecoveryVuMark.CENTER
            //For CENTER and UNKNOWN do the center so that we can get it in somewhere
            if (isRed) {
                //Red alliance and far
                if (isJewelBlue == null) {
                    strafeDistance = strafeDistance + 9;
                }   else if (isJewelBlue.booleanValue()) {
                    //Column tested good
                    //Blue Jewel, This is where we turn to knock off jewel
                    strafeDistance = strafeDistance + 8;
                }   else {
                    //Column tested good
                    //Red Jewel, This is where we go straight to knock off jewel
                    strafeDistance = strafeDistance + 9;
                }
            }   else {
                //Blue alliance and far
                if (isJewelBlue == null) {
                    //TODO not tested
                    strafeDistance = strafeDistance + 6;
                }   else if (isJewelBlue.booleanValue()) {
                    //TODO not tested
                    //Blue Jewel, This is where we turn to knock off jewel
                    strafeDistance = strafeDistance + 6;//Was 5
                }   else {
                    //TODO not tested
                    //Red Jewel, This is where we go straight to knock off jewel
                    strafeDistance = strafeDistance + 7;//Good
                }
            }
        }
        return strafeDistance;
    }

    // Linear distance for both near and far
    private int getLinearDistance(RelicRecoveryVuMark glyphColumnKey, boolean isRed, boolean isNear, Boolean isJewelBlue)
            throws InterruptedException
    {
        //telemetry.addData("Jewel:", "dontKnockOffJewel");
        int linearDistance = isNear ? 22 : 18;
        if (isJewelBlue != null)
        {
            if (!isRed)
            {
                //Blue alliance
                if (isJewelBlue.booleanValue())
                {
                    //Blue Jewel
                    // the red jewel is behind so turn backward
                    linearDistance = linearDistance + JEWEL_DISTANCE + (isNear ? 5 : 2);
                }
                else
                {
                    //Red Jewel
                    // the red jewel is in front so move
                }
            }
            else
            {
                //Red alliance
                if (isJewelBlue.booleanValue())
                {
                    //Blue Jewel
                    // remember the robot is facing backwards
                    // the blue jewel is in front so turn forward
                    linearDistance = linearDistance + JEWEL_DISTANCE + (isNear ? -1 : 2);
                }
                else
                {
                    //Red Jewel
                    // remember the robot is facing backwards
                    // the blue jewel is behind the robot so move backward
                    linearDistance = linearDistance + (isNear ? 0 : -2);
                }
            }
        }
        else
        {
            //Since we are not doing Jewel we should add the distance of travelDistance
            linearDistance = linearDistance + JEWEL_DISTANCE;
        }

        if (isNear)
        {
            if (glyphColumnKey == RelicRecoveryVuMark.LEFT)
            {
                if (isRed)
                {
                    if (isJewelBlue == null)
                    {
                        //TODO not tested
                        linearDistance = linearDistance + 10;
                    }
                    else if (isJewelBlue.booleanValue())
                    {
                        linearDistance = linearDistance + 15;
                    }
                    else
                    {
                        //Tested good
                        linearDistance = linearDistance + 10;
                    }
                }
                else
                {
                    if (isJewelBlue == null)
                    {
                        //TODO not tested
                        linearDistance = linearDistance - 3;
                    }
                    else if (isJewelBlue.booleanValue())
                    {
                        //Tested good
                        linearDistance = linearDistance - 3;
                    }
                    else
                    {
                        //Tested good
                        linearDistance = linearDistance - 3;
                    }
                }
            }
            else if (glyphColumnKey == RelicRecoveryVuMark.RIGHT)
            {
                if (isRed)
                {
                    if (isJewelBlue == null)
                    {
                        //TODO not tested
                        //linearDistance = linearDistance + 0;
                    }
                    else if (isJewelBlue.booleanValue())
                    {
                        //Tested good
                        linearDistance = linearDistance + 2;
                    }
                    else
                    {
                        //Tested good
                        linearDistance = linearDistance - 2;
                    }
                }
                else
                {
                    if (isJewelBlue == null)
                    {
                        //TODO not tested
                        linearDistance = linearDistance + 12;
                    }
                    else if (isJewelBlue.booleanValue())
                    {
                        //Tested good
                        linearDistance = linearDistance + 12;
                    }
                    else
                    {
                        //Tested good
                        linearDistance = linearDistance + 12;
                    }
                }
            }
            else
            {
                //RelicRecoveryVuMark.CENTER
                //For CENTER and UNKNOWN do the center so that we can get it in somewhere
                if (isRed)
                {
                    if (isJewelBlue == null)
                    {
                        //TODO not tested
                        linearDistance = linearDistance + 6;
                    }
                    else if (isJewelBlue.booleanValue())
                    {
                        //Tested good
                        linearDistance = linearDistance + 10;
                    }
                    else
                    {
                        //Tested good
                        linearDistance = linearDistance + 5;
                    }
                }
                else
                {
                    if (isJewelBlue == null)
                    {
                        //TODO not tested
                        linearDistance = linearDistance + 5;
                    }
                    else if (isJewelBlue.booleanValue())
                    {
                        //Tested good
                        linearDistance = linearDistance + 5;
                    }
                    else
                    {
                        //Tested good
                        linearDistance = linearDistance + 5;
                    }
                }
            }
        }
        return linearDistance;
    }


    // Turn angle for near
    private int getNearReturnAngle(RelicRecoveryVuMark glyphColumnKey, boolean isRed, boolean isNear, Boolean isJewelBlue, int turnAngle)
            throws InterruptedException
    {

        int turnAngleReturn = 0;

        if (isRed) {
            //Red alliance
            turnAngleReturn = -turnAngle + (54 / 4);
            if (glyphColumnKey == RelicRecoveryVuMark.LEFT) {
                if (isJewelBlue == null) {
                    //TODO: Needs testing

                } else if (isJewelBlue.booleanValue()){
                    //Tested: good

                } else {
                    //Tested: good
                    turnAngleReturn -= 3;

                }
            } else if (glyphColumnKey == RelicRecoveryVuMark.CENTER) {
                if (isJewelBlue == null) {
                    //TODO: Needs testing

                } else if (isJewelBlue.booleanValue()){
                    //TODO: Needs testing
                    turnAngleReturn -= 3;

                } else {
                    //TODO: Needs testing

                }
            } else if (glyphColumnKey == RelicRecoveryVuMark.RIGHT) {
                if (isJewelBlue == null) {
                    //TODO: Needs testing

                } else if (isJewelBlue.booleanValue()){
                    //TODO: Needs testing

                } else {
                    //TODO: Needs testing

                }
            }
        } else {
            //Blue alliance
            turnAngleReturn = turnAngle - (54 / 4);
            if (glyphColumnKey == RelicRecoveryVuMark.LEFT) {
                if (isJewelBlue == null) {
                    //TODO: Needs testing

                } else if (isJewelBlue.booleanValue()){
                    //TODO: Needs testing
                    turnAngleReturn += 3;

                } else {
                    //TODO: Needs testing

                }
            } else if (glyphColumnKey == RelicRecoveryVuMark.CENTER) {
                if (isJewelBlue == null) {
                    //TODO: Needs testing

                } else if (isJewelBlue.booleanValue()){
                    //TODO: Needs testing

                } else {
                    //TODO: Needs testing

                }
            } else if (glyphColumnKey == RelicRecoveryVuMark.RIGHT) {
                if (isJewelBlue == null) {
                    //TODO: Needs testing

                } else if (isJewelBlue.booleanValue()){
                    //TODO: Needs testing

                } else {
                    //TODO: Needs testing

                }
            }
        }
        return turnAngleReturn;
    }

    /**
     * autonomous function
     * @throws InterruptedException
     */
    public void run ()
            throws InterruptedException
    {
        SelectLinear sl = new SelectLinear(this);
        boolean isRed = sl.selectAlliance();
        boolean isNear = sl.selectPosition();

        //int waitMillis = sl.adjustDelay();

        telemetry.addData("Alliance", isRed ? "Red" : "Blue");
        telemetry.addData("Position", isNear ? "Near" : "Far");

        //telemetry.addData("Delay", Integer.toString(waitMillis/1000));
        telemetry.addData("Waiting", "AutonomousOvercharged");
        telemetry.update();

        if (isRed)
        {
            robot.ledRed.on();
        }
        else
        {
            robot.ledBlue.on();
        }
        WaitLinear lp = new WaitLinear(this);
        //lp.waitMillis(waitMillis);

        robot.ledGreen.set(isNear);
        robot.drawLed();
        //// force encoder reset to 0 regardless whether touchiing the bottom switch
        //robot.slideLeft.resetPosition();
        //robot.slideRight.resetPosition();
        robot.slide.moveToBottom();
        robot.gyroSensor.resetHeading();

        sensorColor = new RevColorDistanceSensor(hardwareMap, "jewel_color");

        waitForStart();

        robot.ledRed.off();
        robot.ledBlue.off();
        robot.ledGreen.off();
        robot.ledWhite.off();
        robot.ledYellow.off();
        robot.drawLed();

        long startTime = System.currentTimeMillis();

        // read color sensor
        robot.servoJewel.setPosition(RobotOverchargedConstants.SERVO_JEWEL_OUT);
        robot.servoClawLeft.setPosition(RobotOverchargedConstants.SERVO_CLAW_LEFT_IN);
        robot.servoClawRight.setPosition(RobotOverchargedConstants.SERVO_CLAW_RIGHT_IN);
        lp.waitMillis(100);
        robot.slide.moveToEncoder(15);
        //lp.waitMillis(500);
        RelicRecoveryVuMark glyphColumnKey = RelicRecoveryVuMark.UNKNOWN;
        try
        {
            glyphColumnKey = robot.relicRecoveryVuMark.getGlyphColumnKey();
        }
        catch (Exception e) { }
        telemetry.addData("GlyphColumnKey:", "%s", glyphColumnKey);

        Boolean isJewelBlue = null;
        int i = 1;
        for (i = 1; i < 10; i++)
        {
            lp.waitMillis(JEWEL_OUT_WAIT_TIME);
            RevColorDistanceSensor.COLORTYPE jewelColor = sensorColor.getColor();
            lp.waitMillis(100);
            if (jewelColor == RevColorDistanceSensor.COLORTYPE.BLUE)
            {
                isJewelBlue = Boolean.TRUE;
                break;
            }
            else if (jewelColor == RevColorDistanceSensor.COLORTYPE.RED)
            {
                isJewelBlue = Boolean.FALSE;
                break;
            }
            robot.servoJewel.setPosition(RobotOverchargedConstants.SERVO_JEWEL_OUT - i * 3 / 255f);
        }
        if (i > 1)
        {
            //reset the servo back to out only if you did not succeed in reading the first time i.e. i > 1
            // in other words if you read it the first time when servoJewel = RobotOverchargedConstants.SERVO_JEWEL_OUT then no need to do it again
            robot.servoJewel.setPosition(RobotOverchargedConstants.SERVO_JEWEL_OUT);
            //there is already a wait below so no need to do this
            //lp.waitMillis(JEWEL_OUT_WAIT_TIME);
        }
        telemetry.addData("Jewel:", isJewelBlue == null ? "Null" : isJewelBlue.booleanValue() ? "blue" : "red");

        int insertAngle = !isNear ? 0 : isRed ? 8 : -8;
        if (glyphColumnKey == RelicRecoveryVuMark.LEFT)
        {
            insertAngle = !isNear ? 0 : isRed ? 15 : -15;
        }

        int linearDistance = getLinearDistance(glyphColumnKey, isRed, isNear, isJewelBlue);
        telemetry.addData("Distance Linear: ", linearDistance);
        telemetry.update();

        robot.slide.moveToEncoder(isRed ? 500 : 650);
        lp.waitMillis(750);
        //Do the Jewel only
        if (isJewelBlue != null) {
            if (!isRed) {
                //Blue alliance
                if (isJewelBlue.booleanValue()) {
                    //Blue Jewel
                    // the red jewel is behind so turn backward
                    knockOffJewelWithTurn(lp, true);
                } else {
                    //Red Jewel
                    // the red jewel is in front so move
                    knockOffJewelWithMove(lp, true);
                }
            } else {
                //Red alliance
                if (isJewelBlue.booleanValue()) {
                    //Blue Jewel
                    // remember the robot is facing backwards
                    // the blue jewel is in front so turn forward
                    knockOffJewelWithTurn(lp, false);
                } else {
                    //Red Jewel
                    // remember the robot is facing backwards
                    // the blue jewel is behind the robot so move backward
                    knockOffJewelWithMove(lp, false);
                }
            }
        } else {
            //TODO test this
            //Since we are not doing Jewel we should add the distance of travelDistance
            dontKnockOffJewel(lp);
        }

        drive.moveToEncoderInch(TurnType.FORWARD,
                isRed ? -linearDistance : linearDistance,
                0.6f,
                30000,
                true,
                true,
                true);

        if (!isNear && isRed)
        {
            //Red and far
            //TODO Change this to the other way of turn
            drive.turn(TurnType.TURN_REGULAR,
                    170,
                    0.20f,
                    -1,
                    true);
        }

        if (!isNear)
        {
            //Strafe for far
            int strafeDistance = getFarStrafeDistance(glyphColumnKey, isRed, isNear, isJewelBlue);
            telemetry.addData("Distance Strafe: ", strafeDistance);
            telemetry.update();
            drive.moveToEncoderInch(TurnType.STRAFE,
                    isRed ? -strafeDistance : strafeDistance,
                    0.5f,
                    30000,
                    false,
                    false,
                    true);
        }
        else
        {
            //Near and turn
            drive.turn(TurnType.TURN_REGULAR,
                    80,
                    0.20f,
                    -1,
                    true);
        }

        //if (insertAngle != 0)
        if (insertAngle > 0)
        {
            drive.turn(TurnType.TURN_REGULAR,
                    insertAngle,
                    0.20f,
                    -1,
                    true);
        }
        //Move into the cryptobox
        drive.moveToTime(TurnType.FORWARD,
                0.25f,
                1000);

        //Open claw
        robot.servoClawLeft.setPosition(RobotOverchargedConstants.SERVO_CLAW_LEFT_OUT);
        robot.servoClawRight.setPosition(RobotOverchargedConstants.SERVO_CLAW_RIGHT_OUT);
        //wait for the open to complete
        lp.waitMillis(CLAW_WAIT_TIME);
        int backoutDistance = getBackoutDistance(glyphColumnKey, isRed, isNear, isJewelBlue);
        //back out
        drive.moveToEncoderInch(TurnType.FORWARD,
                -backoutDistance,
                0.5f,
                30000,
                false,
                false,
                true);

        if (isNear)
        {
            //We will attempt second glyph only for near
            int turnAngle = isRed ? 170 : -165;
            int turnAngleReturn = getNearReturnAngle(glyphColumnKey, isRed, isNear, isJewelBlue, turnAngle);

            telemetry.addData("turnAngle: ", turnAngle);
            telemetry.addData("turnAngleReturn: ", turnAngleReturn);
            int secondGlyphDistance = 36 - backoutDistance;

            //Now go for the second Glyph
            //Move slide to bottom
            robot.slide.moveToBottom();
            //turn to go to the glyph pit
            drive.turn(TurnType.TURN_REGULAR,
                    turnAngle,
                    0.20f,
                    -1,
                    true);
            //move forward to the glyph pit
            drive.moveToEncoderInch(TurnType.FORWARD,
                    secondGlyphDistance,
                    0.5f,
                    30000,
                    false,
                    false,
                    true);

            lp.waitMillis(100);
            //Close the claw hopefully we caught some glyphs if lucky
            robot.servoClawLeft.setPosition(RobotOverchargedConstants.SERVO_CLAW_LEFT_IN);
            robot.servoClawRight.setPosition(RobotOverchargedConstants.SERVO_CLAW_RIGHT_IN);
            //wait for the claw to close completely
            lp.waitMillis(CLAW_WAIT_TIME);
            //Move the slide a bit above the second row level
            robot.slide.moveToEncoder(950);
            lp.waitMillis(CLAW_WAIT_TIME);
            //go to cryptobox after grabbing
            //travel backwards to the cryptobox
            drive.moveToEncoderInch(TurnType.FORWARD,
                    -(secondGlyphDistance / 2),
                    0.4f,
                    30000,
                    false,
                    false,
                    true);
            //turn and face the cryptobox
            drive.turn(TurnType.TURN_REGULAR,
                    turnAngleReturn,
                    0.20f,
                    -1,
                    true);
            //travel completely into the cryptobox
            drive.moveToEncoderInch(TurnType.FORWARD,
                    (secondGlyphDistance / 2) + backoutDistance,
                    0.4f,
                    30000,
                    false,
                    false,
                    true);
            //open the claw
            robot.servoClawLeft.setPosition(RobotOverchargedConstants.SERVO_CLAW_LEFT_OUT);
            robot.servoClawRight.setPosition(RobotOverchargedConstants.SERVO_CLAW_RIGHT_OUT);
            lp.waitMillis(CLAW_WAIT_TIME);
            //this is to back out and not touch the glyph
            int secondBackoutDistance = getSecondBackoutDistance(glyphColumnKey, isRed, isNear, isJewelBlue);
            drive.moveToEncoderInch(TurnType.FORWARD,
                    -secondBackoutDistance,
                    0.5f,
                    30000,
                    false,
                    false,
                    true);
        }
        telemetry.update();
        RobotLog.i("AutonomousOvercharged finish in " + (System.currentTimeMillis() - startTime) + " milliseconds");
        lp.waitMillis(5000);
    }
}