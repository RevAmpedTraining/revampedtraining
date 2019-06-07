package org.firstinspires.ftc.teamcode.Test;

import com.revAmped.components.Button;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "OverchargedAlignTest2", group = "Test")
public class AlignTest2 extends OpMode {
    private ElapsedTime     runtime = new ElapsedTime();
    ///Overcharged Robot class
    //private RobotOvercharged2 robot;
    ///Overcharged Swirve Drive class
    //private SwerveDrive drive;
    ///Ultrasonic sensors used to in the front of the robot
    private ModernRoboticsI2cRangeSensor rangeSensorBL;
    ///Ultrasonic sensors used at the back of the robot
    private ModernRoboticsI2cRangeSensor rangeSensorBR;
    private final static DecimalFormat numberFormatter = new DecimalFormat("######");


    @Override
    public void init() {
        //robot = new RobotOvercharged2(this, false);
        //drive = robot.getSwerveDrive();
        rangeSensorBL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensorBL");
        RobotLog.e("initialized rangeSensorBL");
        rangeSensorBR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensorBR");
        RobotLog.e("initialized rangeSensorBR");
        /**
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.setTurn(TurnType.STRAFE);
        drive.resetPosition();
         **/
    }

    @Override
    public void stop() {
        //drive.setTurn(TurnType.FORWARD);
        //robot.close();
    }

    @Override
    public void loop() {
        long timestamp = System.currentTimeMillis();

        if (gamepad1.start && Button.BTN_START.canPress(timestamp)) {
            //drive.resetPosition();
        }
        /// Validate the robot position near the column
        double distanceL = rangeSensorBL.getDistance(DistanceUnit.CM);
        double distanceR = rangeSensorBL.getDistance(DistanceUnit.CM);
        telemetry.addData("Reset Motor", "Press Start");
        telemetry.addData("Position", "distanceL=" + distanceL + " distanceR=" + distanceR);
        /**
        telemetry.addData("Front",
                "Left:" + numberFormatter.format(robot.driveLeftFront.getCurrentPosition()) +
                        "Right:" + numberFormatter.format(robot.driveRightFront.getCurrentPosition()));
        telemetry.addData("Back",
                "Left:" + numberFormatter.format(robot.driveLeftBack.getCurrentPosition()) +
                        "Right:" + numberFormatter.format(robot.driveRightBack.getCurrentPosition()));
         **/
        telemetry.update();
        //robot.drawLed();
    }

	/*
    @Override
    public void runOpMode()
            throws InterruptedException {
        try {
            // init
            robot = new RobotOverchargedLinear2(this);
            drive = robot.getSwerveDriveLinear();

            run();
        } finally {
            // shut down
            if (robot != null) {
                robot.close();
            }
        }
    }*/

    /**
     * Autonomous run function
     * @throws InterruptedException
     */
    /**
     public void run()
     throws InterruptedException {

     rangeSensorBL = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensorBL");
     RobotLog.e("initialized rangeSensorBL");
     rangeSensorBR = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangeSensorBR");
     RobotLog.e("initialized rangeSensorBR");

     /// Validate the robot position near the column
     double distanceL = rangeSensorBL.getDistance(DistanceUnit.CM);
     double distanceR = rangeSensorBL.getDistance(DistanceUnit.CM);
     telemetry.addData("Position", "distanceL=" + distanceL + " distanceR=" + distanceR);

     //telemetry.addData("Delay", Integer.toString(waitMillis/1000));
     telemetry.addData("Waiting", "AutonomousOvercharged");
     telemetry.update();
     WaitLinear lp = new WaitLinear(this);
     ///Wait for Start to be pressed
     waitForStart();

     long startTime = System.currentTimeMillis();

     telemetry.update();
     RobotLog.i("AutonomousOvercharged finish in " + (System.currentTimeMillis() - startTime) + " milliseconds");
     lp.waitMillis(5000);
     ///End
     }
     **/

    /**
     /**
     * This method can only tolerates a misaligment of max 4 inches. The robot is expected to perform within that range
     *
     */
    /** private void AlignAtColumn(double distanceL, double distanceR)
     {

     ///do the correction only when the column matches
     double tolerance = 4;
     double offtarget = Math.abs(distanceL - distanceR);
     telemetry.addData("Difference", "offtarget=" + offtarget + " tolerance=" + tolerance);
     if (offtarget > tolerance) {
     ///Correction required only when the distance is greater than the tolerance amount
     int correctionDistance = (int) offtarget / 5;
     if (distanceL < distanceR) {
     //Move left
     telemetry.addData("Move", String.format(Locale.US, "Left %.02f", correctionDistance));
     drive.moveToEncoderInch(TurnType.STRAFE,
     -correctionDistance,
     0.2f,
     30000,
     false,
     false,
     true);
     } else {
     //move right
     telemetry.addData("Move", String.format(Locale.US, "Right %.02f", correctionDistance));
     drive.setTurn(TurnType.STRAFE);
     drive.setPower(x2 * 0.7f,
     x1 * 0.7f,
     TurnType.STRAFE);

     drive.moveToEncoderInch(TurnType.STRAFE,
     correctionDistance,
     0.2f,
     30000,
     false,
     false,
     true);
     }
     }

     }
     **/
}