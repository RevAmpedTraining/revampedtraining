package org.firstinspires.ftc.teamcode.SimpleTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class AviTeleOp extends OpMode {

    private DcMotor driveRF,
                    driveRB,
                    driveLF,
                    driveLB;

    @Override
    public void init() {

        // configuring gamepads
        gamepad1.reset();
        gamepad2.reset();

        gamepad1.setJoystickDeadzone(0.1f);
        gamepad2.setJoystickDeadzone(0.1f);

        // initing motors
        driveRF = hardwareMap.get(DcMotor.class, "rf");
        driveRB = hardwareMap.get(DcMotor.class, "rb");
        driveLF = hardwareMap.get(DcMotor.class, "lf");
        driveLB = hardwareMap.get(DcMotor.class, "lb");

        driveLB.setDirection(DcMotorSimple.Direction.REVERSE);
        driveLF.setDirection(DcMotorSimple.Direction.REVERSE);

        // updating telemetry
        telemetry.addLine("Ready to Start");
        telemetry.update();
    }

    @Override
    public void loop() {
        float x1 = gamepad1.left_stick_x,
              x2 = gamepad1.right_stick_x,
              y1 = gamepad1.left_stick_y,
              y2 = gamepad1.right_stick_y;

        // powering the motors if the conditions are met
        if ((y1 > 0 && y2 > 0) || (y1 < 0 && y2 < 0) || ((Math.signum(y1) != Math.signum(y2)) && (y1 != 0) && (y2 != 0))) {
            driveRF.setPower(y2);
            driveRB.setPower(y2);
            driveLF.setPower(y1);
            driveLB.setPower(y1);
        }

        // updating telemetry
        telemetry.addData("Motor Powers:", "rf=%f, rb=%f, lf=%f, lb=%f", y2, y2, y1, y1);
    }

    @Override
    public void stop() {
        driveRF.close();
        driveRB.close();
        driveLF.close();
        driveLB.close();
    }
}
