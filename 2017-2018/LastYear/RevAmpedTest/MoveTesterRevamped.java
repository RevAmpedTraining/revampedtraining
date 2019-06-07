package org.firstinspires.ftc.teamcode.LastYear.RevAmpedTest;

import com.revAmped.components.Button;
import com.revAmped.config.RobotRevAmpedConstants;
import com.revAmped.linear.components.RobotRevAmpedLinear2;
import com.revAmped.linear.components.MecanumDriveLinear;
import com.revAmped.linear.util.SelectLinear;
import com.revAmped.linear.util.WaitLinear;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Move tester
 */
//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="MoveTesterReVamped2", group="Game")
public class MoveTesterRevamped
    extends LinearOpMode {
    private RobotRevAmpedLinear2 robot = null;

    private MecanumDriveLinear drive;
    private static final Button BTN_ROLLER_OUT = new Button();


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
     *
     * @throws InterruptedException
     */
    public void run()
            throws InterruptedException {

        long timestamp = System.currentTimeMillis();

        SelectLinear sl = new SelectLinear(this);
        boolean isRed = sl.selectAlliance();
        boolean isNear = sl.selectPosition();

        //int waitMillis = sl.adjustDelay();

        telemetry.addData("Alliance", isRed ? "Red" : "Blue");
        telemetry.addData("Position", isNear ? "Near" : "Far");

        //telemetry.addData("Delay", Integer.toString(waitMillis/1000));
        telemetry.addData("Waiting", "AutonomousRevAmped");
        telemetry.update();

        WaitLinear.WakeUp moveMonitorForward = new WaitLinear.WakeUp() {
            public boolean isWakeUp() {
                telemetry.addData("Travel", drive.tickToInch(drive.getEncoder()));
                return false;
            }
        };

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        long startTime = System.currentTimeMillis();

        WaitLinear lp = new WaitLinear(this);


        int turntest = 90;
        if (isRed && isNear) {
            float sonarWallRed1 = robot.sonarL.getDistance();
            lp.waitMillis(200);
            float sonarWallRed2 = robot.sonarL.getDistance();
            float sonarWallRed = (sonarWallRed1 + sonarWallRed2)/2;


            while (sonarWallRed>38) {
                drive.setPower(-0.4f, 0.5f, 0.4f, -0.4f);
                telemetry.addData("distance from wall", sonarWallRed);
                 sonarWallRed1 = robot.sonarL.getDistance();
                lp.waitMillis(50);
                 sonarWallRed2 = robot.sonarL.getDistance();
                 sonarWallRed = (sonarWallRed1 + sonarWallRed2)/2;
                telemetry.update();
            }
            /*drive.turn(-20,
                    0.6f,
                    3000,
                    true);*/

        } else if (!isRed && isNear){
            robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_OUT);
            lp.waitMillis(500);
        }
        else {
            robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
            robot.sweeperRight.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
            robot.sweeperLeft.setPower(RobotRevAmpedConstants.POWER_SWEEPER);
            robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_OUT);
            robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
            lp.waitMillis(50);
            drive.moveToTime(0.35f, 1800);
            lp.waitMillis(1000);
            drive.moveToTime(-0.35f, 500);
            lp.waitMillis(100);
            drive.moveToTime(0.3f, 250);
            robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
            robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_OUT);
            robot.sweeperRight.setPower(-1.0f);
            robot.sweeperLeft.setPower(-1.0f);
            lp.waitMillis(250);
            robot.sweeperRight.setPower(1f);
            robot.sweeperLeft.setPower(1f);
            drive.moveToTime(-0.3f, 250);
            lp.waitMillis(250);
            robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_OUT);
            robot.servoDoorRight.setPosition(RobotRevAmpedConstants.SERVO_DOOR_IN);
            lp.waitMillis(1500);
            drive.moveToTime(-0.35f, 1800);
            robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_FLAT);
            robot.slide.moveToEncoder(1000);//move slide to first level

            lp.waitMillis(100);
        }
    }


}