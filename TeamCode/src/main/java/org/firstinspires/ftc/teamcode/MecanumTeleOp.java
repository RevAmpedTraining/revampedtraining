package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.revAmped.components.Button;

import com.qualcomm.robotcore.util.Range;



/**
 * Created by swang4 on 6/11/2019.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="MecanumTeleOp", group="Example")
public class MecanumTeleOp extends OpMode{

    private DcMotor driveLeftFront = null;
    private DcMotor driveRightFront = null;
    private DcMotor driveLeftBack = null;
    private DcMotor driveRightBack = null;

    @Override
    public void init() {
        driveLeftFront = hardwareMap.get(DcMotor.class, "motor_lf");
        driveRightFront = hardwareMap.get(DcMotor.class, "motor_rf");
        driveLeftBack = hardwareMap.get(DcMotor.class, "motor_lb");
        driveRightBack = hardwareMap.get(DcMotor.class, "motor_rb");

        this.gamepad1.reset();
        this.gamepad1.setJoystickDeadzone(0.15f);
        this.gamepad2.reset();
        this.gamepad2.setJoystickDeadzone(0.15f);
    }
    @Override
    public void loop() {
        float x1 = Range.clip(Button.scaleInput(gamepad1.left_stick_x), -1f, 1f);
        float y1 = Range.clip(Button.scaleInput(gamepad1.left_stick_y), -1f, 1f);
        float x2 = Range.clip(Button.scaleInput(-gamepad1.right_stick_x), -1f, 1f);
        float y2 = Range.clip(Button.scaleInput(gamepad1.right_stick_y), -1f, 1f);

        if ((Math.abs(y1) < Math.abs(x1) && Math.abs(y2) < Math.abs(x2))/* ||
                (Math.abs(y1) < Math.abs(x1) && x2 == 0) ||
                (Math.abs(y2) < Math.abs(x2) && x1 == 0)*/) {

            float avg = (x1 + x2)/2f;

            driveLeftBack.setPower(-avg);
            driveRightFront.setPower(avg);
            driveLeftFront.setPower(avg);
            driveRightBack.setPower(-avg);
        }
        //straight tank
        else {

            driveLeftBack.setPower(-y1);
            driveRightFront.setPower(y2);
            driveLeftFront.setPower(-y1);
            driveRightBack.setPower(y2);
        }
    }
    @Override
    public void stop() {}

}
