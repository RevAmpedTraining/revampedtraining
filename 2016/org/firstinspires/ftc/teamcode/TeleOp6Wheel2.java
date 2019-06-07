package org.firstinspires.ftc.teamcode;

import com.revAmped.components.Button;
import com.revAmped.components.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * TeleOp
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp6Wheel", group="Game")
public class TeleOp6Wheel2
        extends OpMode {
    
    private Robot robot;
    
    public DcMotor driveLeftFront = null;
    public DcMotor driveRightFront = null;
    public DcMotor driveLeftBack = null;
    public DcMotor driveRightBack = null;

    public enum EEncoderStatus
    {
        PRE_USE_ENCODER,
        USE_ENCODER,
        PRE_WITHOUT_ENCODER,
        WITHOUT_ENCODER;
    }

    @Override
    public void init() {
        driveLeftFront = hardwareMap.dcMotor.get("driveLF");
        driveLeftBack = hardwareMap.dcMotor.get("driveLB");
        driveRightFront = hardwareMap.dcMotor.get("driveRF");
        driveRightBack = hardwareMap.dcMotor.get("driveRB");

        driveLeftFront.setDirection(DcMotor.Direction.FORWARD);
        driveLeftBack.setDirection(DcMotor.Direction.FORWARD);
        driveRightFront.setDirection(DcMotor.Direction.REVERSE);
        driveRightBack.setDirection(DcMotor.Direction.REVERSE);

        driveLeftFront.setPower(0);
        driveLeftBack.setPower(0);
        driveRightFront.setPower(0);
        driveRightBack.setPower(0);

        this.gamepad1.reset();
        this.gamepad1.setJoystickDeadzone(0.15f);
        this.gamepad2.reset();
        this.gamepad2.setJoystickDeadzone(0.15f);
    }

    @Override
    public void stop() {
        driveLeftFront.setPower(0);
        driveLeftBack.setPower(0);
        driveRightFront.setPower(0);
        driveRightBack.setPower(0);
    }

    @Override
    public void loop() {
        // is reversed by default
        float x1 = gamepad1.left_stick_x;
        float y1 = -gamepad1.left_stick_y;
        float x2 = gamepad1.right_stick_x;
        float y2 = -gamepad1.right_stick_y;

        float tx1 = gamepad2.left_stick_x;
        float ty1 = -gamepad2.left_stick_y;
        float tx2 = gamepad2.right_stick_x;
        float ty2 = -gamepad2.right_stick_y;

        x1 = Button.scaleInput(x1);
        x2 = Button.scaleInput(x2);
        y1 = Button.scaleInput(y1);
        y2 = Button.scaleInput(y2);

        // drive
        x1 = Range.clip(x1, -1f, 1f);
        x2 = Range.clip(x2, -1f, 1f);
        y1 = Range.clip(y1, -1f, 1f);
        y2 = Range.clip(y2, -1f, 1f);

        driveLeftFront.setPower(y1);
        driveLeftBack.setPower(y1);
        driveRightFront.setPower(y2);
        driveRightBack.setPower(y2);
    }
}
