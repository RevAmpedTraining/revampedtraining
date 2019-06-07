package org.firstinspires.ftc.teamcode.LastYear.RevAmpedTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;
import com.revAmped.components.Button;
import com.revAmped.components.HwMotor;
import com.revAmped.linear.components.MecanumDriveLinear;
import com.revAmped.linear.components.RobotRevAmpedLinear2;

import java.text.DecimalFormat;

/**
 * Created by swang4 on 4/19/2018.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Motor Test", group="Test")
public class DriveTest extends LinearOpMode {
    public DcMotor frontLeftMotor = null;
    public DcMotor backLeftMotor = null;
    private final static DecimalFormat numberFormatter = new DecimalFormat("######");
    private MecanumDriveLinear drive;


    @Override
    public void runOpMode()  throws InterruptedException {


        frontLeftMotor = hardwareMap.get(DcMotor.class, "motor_lf");
        backLeftMotor = hardwareMap.get(DcMotor.class, "motor_lb");

    }
    public void run()
    throws InterruptedException{

        for (int i = 0; i < 4; i++) {
            //drive.resetPosition();

            long startTimestamp = System.currentTimeMillis();
            long timeStamp = startTimestamp;

            while (opModeIsActive() &&
                    timeStamp - startTimestamp < 5000) {
                if (i % 2 == 0) {
                    frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
                frontLeftMotor.setPower(0.2f);
                backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
                frontLeftMotor.setPower(0.2f);

            } else {
                    frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
                    frontLeftMotor.setPower(0.2f);
                    backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
                    frontLeftMotor.setPower(0.2f);
                }

                telemetry.addData("Front",
                        "Left:" + numberFormatter.format(frontLeftMotor.getCurrentPosition()) +
                                " Right:" + numberFormatter.format(frontLeftMotor.getCurrentPosition()));
                telemetry.addData("Back",
                        "Left:" + numberFormatter.format(backLeftMotor.getCurrentPosition()) +
                                " Right:" + numberFormatter.format(backLeftMotor.getCurrentPosition()));
                telemetry.addData("Stop", "Back");

                telemetry.update();
                idle();
                timeStamp = System.currentTimeMillis();
            }

            drive.stop();
        }

        drive.stop();
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        idle();
    }
}
