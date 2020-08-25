package org.firstinspires.ftc.teamcode.SimpleTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

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
        float x1 = Range.clip(gamepad1.left_stick_x, -1, 1),
              x2 = Range.clip(gamepad1.right_stick_x, -1, 1),
              y1 = Range.clip(gamepad1.left_stick_y, -1, 1),
              y2 = Range.clip(gamepad1.right_stick_y, -1, 1);

        // moving forward
        /*
        if ((y1 > 0 && y2 > 0) || (y1 < 0 && y2 < 0) || ((Math.signum(y1) != Math.signum(y2)) && (y1 != 0) && (y2 != 0))) {
            driveRF.setPower(y2);
            driveRB.setPower(y2);
            driveLF.setPower(y1);
            driveLB.setPower(y1);
        }
         */

        // strafing
        if (Math.abs(x1) > 0.5f && Math.abs(x2) > 0.5f && Math.abs(y1) < 0.5f && Math.abs(y2) < 0.5f) {

            if (x1 > 0 && x2 > 0) { // strafe right
                driveLF.setPower(x1); // left front motor goes forwards
                driveLB.setPower(-x1); // left back motor goes backwards
                driveRF.setPower(-x2); // right front motor goes backwards
                driveRB.setPower(x2); // right back motor goes forwards
            }
            else if (x1 < 0 && x2 < 0) { // strafe left
                driveLF.setPower(x1); 
                driveLB.setPower(-x1);
                driveRF.setPower(-x2);
                driveRB.setPower(x2);
            }
        }
        else {
            driveLF.setPower(y1);
            driveLB.setPower(y1);
            driveRF.setPower(y2);
            driveRB.setPower(y2);
        }

        // updating telemetry
        telemetry.addData("Motor Powers Y:", "rf=%f, rb=%f, lf=%f, lb=%f", y2, y2, y1, y1);
    }

    @Override
    public void stop() {
        driveRF.close();
        driveRB.close();
        driveLF.close();
        driveLB.close();
    }
}
