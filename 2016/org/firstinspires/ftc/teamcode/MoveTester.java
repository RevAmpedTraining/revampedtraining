package org.firstinspires.ftc.teamcode;

import com.revAmped.components.TurnType;
import com.revAmped.linear.components.RobotLinear;
import com.revAmped.linear.components.SwerveDriveLinear;
import com.revAmped.linear.util.WaitLinear;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Move tester
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="MoveTester", group="Game")
public class MoveTester
    extends LinearOpMode
{
    private RobotLinear robot = null;

    private SwerveDriveLinear drive;

    private float pwrSlide;
    private static final float POWER_ROLLER = 0.9f;
    private static final float POWER_SPINNER = 1f;


    /**
     * autonomous opMode
     */
    @Override
    public void runOpMode()
        throws InterruptedException {
        try {
            // init
            robot = new RobotLinear(this);
            drive = robot.getSwerveDriveLinear();

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

        WaitLinear.WakeUp moveMonitorForward = new WaitLinear.WakeUp() {
            public boolean isWakeUp() {
                telemetry.addData("Travel", drive.tickToInch(drive.getEncoder(TurnType.FORWARD)));
                return false;
            }
        };
        WaitLinear.WakeUp moveMonitorStrafe = new WaitLinear.WakeUp() {
            public boolean isWakeUp() {
                telemetry.addData("Travel", drive.tickToInch(drive.getEncoder(TurnType.STRAFE)));
                return false;
            }
        };

        waitForStart();

        long startTime = System.currentTimeMillis();

        WaitLinear lp = new WaitLinear(this);

        /*//drive until reaching 20 cm
        drive.setTurn(TurnType.STRAFE);
        lp.waitMillis(1000);
        drive.moveToSonar(TurnType.STRAFE, 0.3f, 7f, 10000);*/

        //drive for 3 seconds
        //drive.moveToTime(TurnType.FORWARD, 0.12f, 3000);

        //drive until the white line
        //drive.moveToLine(TurnType.FORWARD, 0.5f, 5000);

        // move in a square
        final int DISTANCE_INCH = 24*3;
        final float TURN_DEGREE = 90;
        //final TurnType TURN_TYPE = TurnType.FORWARD;
        final TurnType TURN_TYPE = TurnType.STRAFE;
        //final int MIN_SPACING = 25; // for FORWARD
        final int MIN_SPACING = 7; // for STRAFE

        /*drive.moveToSonar(TurnType.STRAFE,
                          0.3f,
                          7f,
                          30000,
                          true,
                          true);

        drive.moveToEncoderInch(TurnType.STRAFE,
                                24000,
                                100,
                                30000,
                                6,
                                true,
                                true,
                                true);*/
        // 0
        drive.moveToEncoderInch(TURN_TYPE,
                                DISTANCE_INCH,
                                100,
                                30000,
                                MIN_SPACING,
                                true,
                                true,
                                true);
        drive.turn(TurnType.TURN_REGULAR,
                   TURN_DEGREE,
                   1f,
                   -1,
                   true);
        //90
        drive.moveToEncoderInch(TURN_TYPE,
                                DISTANCE_INCH,
                                100,
                                -1,
                                MIN_SPACING,
                                true,
                                true,
                                true);
        drive.turn(TurnType.TURN_REGULAR,
                   TURN_DEGREE,
                   1f,
                   -1,
                   true);
        // 180 = 2*90
        drive.moveToEncoderInch(TURN_TYPE,
                                DISTANCE_INCH,
                                100,
                                -1,
                                MIN_SPACING,
                                true,
                                true,
                                true);
        drive.turn(TurnType.TURN_REGULAR,
                   TURN_DEGREE,
                   1f,
                   -1,
                   true);
        // 270 = 3*90
        drive.moveToEncoderInch(TURN_TYPE,
                                DISTANCE_INCH,
                                100,
                                -1,
                                MIN_SPACING,
                                true,
                                true,
                                true);
        drive.turn(TurnType.TURN_REGULAR,
                   TURN_DEGREE,
                   1f,
                   -1,
                   true);
    }
}
