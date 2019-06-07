package com.revAmped.test;

import com.revAmped.components.Robot;
import com.revAmped.linear.components.RobotLinear;
import com.revAmped.linear.components.TankDriveLinear;
import com.revAmped.components.TurnType;
import com.revAmped.linear.util.WaitLinear;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Robot movement tests
 */
public class MoveTest
    extends LinearOpMode
{
    @Override
    public void runOpMode()
        throws InterruptedException
    {
        // init
        final RobotLinear robot = new RobotLinear(this);
        final TankDriveLinear drive = robot.getTankDriveLinear();

        waitForStart();

        int waitMillis = 3000;
        float maxPower = Robot.AM40_ENCODER_RATIO;
        WaitLinear.WakeUp moveMonitor = new WaitLinear.WakeUp() {
            public boolean isWakeUp() {
                telemetry.addData("Travel", drive.tickToInch(drive.getEncoder()));
                return false;
            }
        };

        WaitLinear lp = new WaitLinear(this);
 /**
        int distanceInch = 24;
        for (int i = 0; i < 2; i++) {
            drive.moveToEncoderInch(distanceInch,
                                    maxPower,
                                    -1,
                                    true,
                                    true);

            distanceInch = -distanceInch;
            lp.waitMillis(waitMillis,
                          moveMonitor);
        }

        int moveTime = 2000;
        float power = 0.4f;
        for (int i = 0; i < 2; i++) {
            drive.moveToTime(power,
                             moveTime);

            power = -power;
            lp.waitMillis(waitMillis);
        }
 */
        WaitLinear.WakeUp turnMonitor = new WaitLinear.WakeUp() {
            public boolean isWakeUp() {
                telemetry.addData("Heading", drive.gyroSensor.getHeading());
                return false;
            }
        };

        int turnAngle = 35;
        /*
        for (int i = 0; i < 2; i++) {
            drive.turn(turnAngle,
                       maxPower,
                       2500,
                       true);

            turnAngle = -turnAngle;
            lp.waitMillis(waitMillis,
                          turnMonitor);
        }
*/
        turnAngle = 35;
        for (int i = 0; i < 1; i++) {
            drive.turn(turnAngle,
                       0.5f,
                       2000,
                       false,
                       TurnType.TURN_LEFT_PIVOT);

            turnAngle = -turnAngle;
            lp.waitMillis(waitMillis,
                          turnMonitor);
        }

        turnAngle = -35;
        for (int i = 0; i < 1; i++) {
            drive.turn(turnAngle,
                       0.5f,
                       2000,
                       false,
                       TurnType.TURN_RIGHT_PIVOT);

            //turnAngle = -turnAngle;
            lp.waitMillis(waitMillis,
                          turnMonitor);
        }
    }
}