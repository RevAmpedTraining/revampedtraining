package org.firstinspires.ftc.teamcode.Robotin30Code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;
import com.revAmped.components.Button;
import com.revAmped.linear.components.RobotRevAmpedLinear;
import com.revAmped.linear.components.TankDriveLinear;
import com.revAmped.linear.util.SelectLinear;
import com.revAmped.linear.util.WaitLinear;
import com.revAmped.linear.components.MoveAction;
import com.revAmped.sensors.RevColorDistanceSensor;
import com.revAmped.util.GlyphDetector;
import com.revAmped.util.MoveActionImpl;


/**
 * Created by John Wang on 9/2/2018.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Robot in 30 Hours Auto", group="Test")
public class Rin30Auto
    extends LinearOpMode{

    private RobotRevAmpedLinear robot;

    private TankDriveLinear drive;

    private static final Button BTN_DELAY_ADD = new Button();
    private static final Button BTN_DELAY_MINUS = new Button();

    MoveAction[] actionsList = new MoveAction[] {new MoveActionImpl(robot)};


    @Override
    public void runOpMode()
        throws InterruptedException {
        try {
            // init
            robot = new RobotRevAmpedLinear(this);
            drive = robot.getTankDriveLinear();
            run();
        } finally {
            // shut down
            if (robot != null) {
                robot.close();
            }
        }
    }

    public void run()
        throws InterruptedException{

        int waitDelay = 0;
        while (!gamepad1.x) {
            long timestamp = System.currentTimeMillis();
            if (gamepad1.right_trigger>0.9f && BTN_DELAY_ADD.canPress(timestamp)) {
                waitDelay+=500;
            } else if (gamepad1.left_trigger>0.9f && BTN_DELAY_MINUS.canPress(timestamp)) {
                waitDelay-=500;
            }
            if (waitDelay<0) {
                waitDelay=0;
            } else if (waitDelay>10000) {
                waitDelay=10000;
            }
            telemetry.addData("Wait Delay", waitDelay);
            telemetry.update();
        }
        //Pick Alliance/Position of Match
        SelectLinear sl = new SelectLinear(this);
        Boolean isRed = sl.selectAlliance();
        Boolean isLeft = sl.selectPosition();

        telemetry.addData("Alliance", isRed);
        telemetry.addData("Position", isLeft);
        telemetry.addData("Wait Delay", waitDelay);
        telemetry.update();

        if (isRed) {
            robot.ledRed.on();
        } else {
            robot.ledBlue.on();
        }

        WaitLinear lp = new WaitLinear(this);

        robot.ledGreen.on();
        robot.drawLed();

        robot.motorSlide.resetPosition();
        robot.gyroSensor.resetHeading();

        waitForStart();

        robot.ledRed.off();
        robot.ledBlue.off();
        robot.ledGreen.off();
        robot.ledWhite.off();
        robot.ledYellow.off();
        robot.drawLed();

        long startTime = System.currentTimeMillis();
        /*
            Land from latch
         */
        lp.waitMillis(waitDelay);
        if (isRed&&!isLeft || !isRed && isLeft) {
            //turn so the front faces the wall
            drive.turn(-30,
                    0.4f,
                    3000,
                    true);
            lp.waitMillis(250);
            //For knocking the gold thingy-don't do
            /*Boolean gold = false;
            if (gold) {
                Boolean isWhite = robot.revColorDistanceSensor.isWhite();
                telemetry.addData("White", isWhite);
                telemetry.update();
                if (isWhite) {
                    int i = 1;
                    drive.moveToEncoderInch(14,
                            0.5f,
                            3000,
                            true,
                            false,
                            true);
                    isWhite = robot.revColorDistanceSensor.isWhite();
                    if (isWhite) {
                        i++;
                        drive.moveToEncoderInch(15,
                                0.5f,
                                3000,
                                true,
                                false,
                                true);
                        drive.turn(20,
                                0.4f,
                                3000,
                                false);
                    } else {
                        drive.turn(20,
                                0.4f,
                                3000,
                                false);
                    }
                } else {
                    drive.turn(20,
                            0.4f,
                            3000,
                            false);
                }
            }
            */
            //drive into the wall to make the robot straight
            drive.moveToEncoderInch(36,
                    0.3f,
                    6000,
                    true,
                    false,
                    true,
                    null);
            lp.waitMillis(500);
            //turn 90 degrees
            drive.turn(-83,
                    0.35f,
                    5000,
                    true);
            lp.waitMillis(250);
            //drive to team marker place
            drive.moveToEncoderInch(-48,
                    0.5f,
                    4000,
                    true,
                    false,
                    true,
                    null);
            /*
            Drop team marker
            */
            //drive to parking
            drive.moveToEncoderInch(108,
                    0.6f,
                    10000,
                    true,
                    false,
                    true,
                    null);
        }




        RobotLog.i("AutonomousRevamped finish in " + (System.currentTimeMillis() - startTime) + " milliseconds");
        lp.waitMillis(1000);

    }

}
