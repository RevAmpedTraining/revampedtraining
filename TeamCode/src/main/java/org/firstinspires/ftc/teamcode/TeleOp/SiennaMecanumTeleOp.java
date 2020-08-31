package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.revAmped.components.Button;
import com.revAmped.components.MecanumDrive;
import com.revAmped.components.RobotEncoderTest;

@TeleOp(name = "Sienna TeleOp", group = "TeleOp")
public class SiennaMecanumTeleOp extends OpMode {

    private RobotEncoderTest robot;
    private MecanumDrive drive;

    private boolean slowMode = false;
    private boolean tankMode = false;

    private Button slow = new Button();
    private Button tank = new Button();

    private final float SLOW_MULT = 0.4f;

    @Override
    public void init() {

        robot = new RobotEncoderTest(this, false);
        drive = robot.getMecanumDrive();

        gamepad1.reset();
        gamepad1.setJoystickDeadzone(0.1f);
        gamepad2.reset();
        gamepad2.setJoystickDeadzone(0.1f);

        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready to start");
        telemetry.update();

    }

    @Override
    public void loop() {

        long timestamp = System.currentTimeMillis();

        float x1 = Range.clip(gamepad1.left_stick_x, -1, 1);
        float y1 = Range.clip(gamepad1.left_stick_y, -1, 1);
        float x2 = Range.clip(gamepad1.right_stick_x, -1, 1);
        float y2 = Range.clip(gamepad1.right_stick_x, -1, 1);

        if(slowMode) {
            x1 *= SLOW_MULT;
            x2 *= SLOW_MULT;
            y1 *= SLOW_MULT;
            y2 = SLOW_MULT;
        }

        if (gamepad1.a && slow.canPress(timestamp)) slowMode = !slowMode;
        if (gamepad1.b && tank.canPress(timestamp)) tankMode = !tankMode;

        if (tankMode) {
            drive.setPower(-y1, y2);
        }
        else {
            if (Math.abs(x1) > 0.25 && Math.abs(y1) < 0.1 && Math.signum(x1) == Math.signum(x2)) {
                drive.setStrafePower((x1+x2) / 2);
            }
            else {
                drive.setPower(-y1, y2);
            }
        }

    }

    @Override
    public void stop() {

    }
}
