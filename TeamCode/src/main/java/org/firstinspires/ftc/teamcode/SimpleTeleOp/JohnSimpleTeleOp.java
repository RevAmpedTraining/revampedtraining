package org.firstinspires.ftc.teamcode.SimpleTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class JohnSimpleTeleOp extends OpMode {

    private DcMotor driveRF;
    private DcMotor driveRB;
    private DcMotor driveLF;
    private DcMotor driveLB;

    @Override
    public void init() {
        gamepad1.reset();
        gamepad1.setJoystickDeadzone(0.1f);
        gamepad2.reset();
        gamepad2.setJoystickDeadzone(0.1f);
        //initialize motors
        driveRF = hardwareMap.get(DcMotor.class, "rf");
        driveRB = hardwareMap.get(DcMotor.class, "rb");
        driveLF = hardwareMap.get(DcMotor.class, "lf");
        driveLB = hardwareMap.get(DcMotor.class, "lb");
        //reverse one side so both sides are aligned
        driveLB.setDirection(DcMotorSimple.Direction.REVERSE);
        driveLF.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Ready to start");
        telemetry.update();
    }
    @Override
    public void loop() {
        //get gamepad inputs
        float x1 = Range.clip(gamepad1.left_stick_x, -1, 1);
        float y1 = Range.clip(gamepad1.left_stick_y, -1, 1);
        float x2 = Range.clip(gamepad1.right_stick_x, -1, 1);
        float y2 = Range.clip(gamepad1.right_stick_y, -1, 1);
        //since y1 and y2 are both positive, we want to go forward
        /*if (y1 > 0 && y2 > 0) {
            driveRF.setPower(y2);
            driveRB.setPower(y2);
            driveLF.setPower(y1);
            driveLB.setPower(y1);
        }
        //if y1 and y2 are both negative, we want to go backward
        else if (y1 < 0 && y2 < 0) {
            driveRF.setPower(y2);
            driveRB.setPower(y2);
            driveLF.setPower(y1);
            driveLB.setPower(y1);
        }
        //if y1 and y2 are opposite signs, we want to turn
        else if ((Math.signum(y1) != Math.signum(y2)) && (y1 != 0) && (y2 != 0)) {
            driveRF.setPower(y2);
            driveRB.setPower(y2);
            driveLF.setPower(y1);
            driveLB.setPower(y1);
        }*/

        if (/*if we want to strafe, then enter this block*/Math.abs(x1) > 0.5f && Math.abs(x2) > 0.5f && Math.abs(y1) < 0.5f && Math.abs(y2) < 0.5f) {
            //strafing code
            //if x1 is positive and x2 is positive, then strafe right
            //if both are negative, strafe left

            //driveRF.setPower();
            //driveRB.setPower();
            //driveLF.setPower();
            //driveLB.setPower();
        } else {
            driveRF.setPower(y2);
            driveRB.setPower(y2);
            driveLF.setPower(y1);
            driveLB.setPower(y1);
        }

        telemetry.addData("Motor powers", "rf=%f, rb=%f, lf=%f, lb=%f", y2, y2, y1, y1);
    }
    @Override
    public void stop() {
        driveRF.close();
        driveRB.close();
        driveLF.close();
        driveLB.close();
    }
}
