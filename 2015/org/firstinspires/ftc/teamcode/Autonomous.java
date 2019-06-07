package org.firstinspires.ftc.teamcode;

import com.revAmped.components.Robot;
import com.revAmped.components.TurnType;
import com.revAmped.config.RobotConstants;
import com.revAmped.linear.components.MoveAction;
import com.revAmped.linear.components.RobotLinear;
import com.revAmped.linear.components.TankDriveLinear;
import com.revAmped.linear.util.SelectLinear;
import com.revAmped.linear.util.WaitLinear;
import com.revAmped.util.HwLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * New Autonomous
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Game")
public class Autonomous
    extends LinearOpMode
{
    private RobotLinear robot = null;
    private TankDriveLinear drive;

    public static final float ROLLER_POWER = -0.75f;

    /**
     * tele-op opMode
     */
    @Override
    public void runOpMode()
        throws InterruptedException {
        try {
            // init
            robot = new RobotLinear(this);
            drive = robot.getTankDriveLinear();

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
        SelectLinear sl = new SelectLinear(this);
        boolean isRed = sl.selectAlliance();
        boolean isNear = sl.selectPosition();
        boolean isBackoff = sl.confirm("Backoff", true);
        boolean isBackoffNear;
        if(isBackoff) {
            isBackoffNear = sl.selectBackoff();
        }
        else {
            isBackoffNear = true;
        }
        boolean isPushBeacon = sl.confirm("Push Beacon", true);
        int waitMillis = sl.adjustDelay();

        telemetry.addData("Alliance", isRed ? "Red" : "Blue");
        telemetry.addData("Position", isNear ? "Near" : "Far");
        telemetry.addData("Backoff", isBackoff?"Yes" : "No");
        if(isBackoff) {
            telemetry.addData("Backoff Distance", isBackoffNear ? "Near" : "Far");
        }
        telemetry.addData("Push Beacon", isPushBeacon ? "Yes" : "No");
        telemetry.addData("Delay", Integer.toString(waitMillis/1000));
        telemetry.addData("Waiting", "Autonomous");

        if (isRed) {
            robot.ledRed.on();
        }
        else {
            robot.ledBlue.on();
        }
        robot.ledGreen.set(!isNear);
        robot.ledWhite.set(isBackoff);
        robot.ledYellow.set(isBackoffNear);
        robot.drawLed();

        waitForStart();

        long startTime = System.currentTimeMillis();
        robot.ledRed.off();
        robot.ledBlue.off();
        robot.ledGreen.off();
        robot.ledWhite.off();
        robot.ledYellow.off();
        robot.drawLed();

        WaitLinear lp = new WaitLinear(this);
        lp.waitMillis(waitMillis);

        robot.roller.setPower(ROLLER_POWER); //positive is out, negative is in
        idle();

        drive.moveToEncoderInch(isNear ? 85 : 112,
                                Robot.AM40_ENCODER_RATIO,
                                isNear ? 8000 : 9000,
                                true,
                                true);

        robot.roller.setPower(0);
        idle();

        drive.turn(isRed ? isNear ? -39 : -31 : isNear ? 39 : 31,
                   0.5f,
                   2000,
                   false,
                   isRed ? TurnType.TURN_LEFT_PIVOT : TurnType.TURN_RIGHT_PIVOT);
        drive.moveToTime(0.35f, 1000);

        robot.servoClimber.setPosition(RobotConstants.SERVO_CLIMBER_OUT);
        idle();
        long startTimestamp = System.currentTimeMillis();
        Boolean bLeftRed = robot.isLeftRed();
        HwLog.i("Autonomous left red: " + bLeftRed);
        if (isPushBeacon && bLeftRed != null) {
            float servoBeaconPosition;
            if (bLeftRed) {
                servoBeaconPosition = isRed ? RobotConstants.SERVO_BEACON_LEFT : RobotConstants.SERVO_BEACON_RIGHT;
            }
            else {
                servoBeaconPosition = isRed ? RobotConstants.SERVO_BEACON_RIGHT : RobotConstants.SERVO_BEACON_LEFT;
            }
            robot.servoBeacon.setPosition(servoBeaconPosition);
            HwLog.i("Autonomous servo beacon: " + servoBeaconPosition);
        }
        lp.waitMillis(1100,
                      startTimestamp);
        robot.servoClimber.setPosition(RobotConstants.SERVO_CLIMBER_IN);
        idle();
        lp.waitMillis(200,
                      startTimestamp);

        HwLog.i("Autonomous dump in " + (System.currentTimeMillis() - startTime) + " milliseconds");
        if (isBackoff) {
            if (isPushBeacon) {
                drive.moveToEncoderInch(-8,
                                        Robot.AM40_ENCODER_RATIO,
                                        500,
                                        false,
                                        true);
            }
            drive.turn(isRed ? -35 : 35,
                       0.5f,
                       2000,
                       false,
                       isRed ? TurnType.TURN_RIGHT_PIVOT : TurnType.TURN_LEFT_PIVOT);
            // wait until 10 seconds to make sure no penalties aren't incurred
            int waitTime = (int) (9700 - System.currentTimeMillis() + startTime);
            if (waitTime > 0) {
                lp.waitMillis(waitTime);
            }
            drive.moveToEncoderInch(isBackoffNear ? -26 : -52,
                                    Robot.AM40_ENCODER_RATIO,
                                    5000,
                                    true,
                                    true,
                                    new MoveAction() {
                                        public int distanceInTick() {
                                            return drive.inchToTick(13);
                                        }

                                        public void perform() {
                                            robot.servoBucket.setPosition(RobotConstants.SERVO_FDOOR_OPEN);
                                            robot.servoBackDoor.setPosition(RobotConstants.SERVO_BDOOR_PUSH);
                                            robot.roller.setPower(-ROLLER_POWER);
                                        }

                                        public void stop() {
                                            robot.roller.setPower(0);
                                            robot.servoBackDoor.setPosition(RobotConstants.SERVO_BDOOR_OPEN);
                                        }
                                    });
        }
        HwLog.i("Autonomous finish in " + (System.currentTimeMillis() - startTime) + " milliseconds");
    }
}
