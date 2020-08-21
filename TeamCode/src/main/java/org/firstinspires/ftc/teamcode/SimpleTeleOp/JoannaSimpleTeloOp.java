package org.firstinspires.ftc.teamcode.SimpleTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class JoannaSimpleTeloOp extends OpMode {
    private DcMotor driveRF;
    private DcMotor driveRB;
    private DcMotor driveLF;
    private DcMotor driveLB;


    @Override
    public void init () {

        gamepad1.reset ();
        gamepad1.setJoystickDeadzone(0.1f);
        gamepad2.reset ();
        gamepad2.setJoystickDeadzone(0.1f);

        driveRF = hardwareMap.get (DcMotor.class, "rf");
        driveRB = hardwareMap.get (DcMotor.class, "rb");
        driveLF = hardwareMap.get (DcMotor.class, "lf");
        driveLB = hardwareMap.get (DcMotor.class, "lb");


        telemetry.addLine("Ready to start");
        telemetry.update();
    }
    @Override
    public void loop () {
        //float x1 = gamepad1.left_stick_x;
        float y1 = gamepad1.left_stick_y;
        //float x2 = gamepad2.right_stick_x;
        float y2 = gamepad2.right_stick_y;

        if (y1 > 0 && y2 >0) {
            driveRF.setPower(y2);
            driveRB.setPower(y2);
            driveLF.setPower(y1);
            driveLB.setPower(y1);

        }else if (y1 < 0 && y2 < 0) {
            driveRF.setPower(y2);
            driveRB.setPower(y2);
            driveLF.setPower(y1);
            driveLB.setPower(y1);


        }

        else if (Math.signum(y1) != Math.signum(y2) && (y1 != 0) && (y2 != 0 ))  {
            driveRF.setPower(y2);
            driveRB.setPower(y2);
            driveLF.setPower(y1);
            driveLB.setPower(y1);

        }

        telemetry.addData("Motor powers", "rf=%f, rb=%f, lf=%f, lb=%f", y2, y2, y1, y1);


    }
    @Override
    public void stop () {
        driveRF.close();
        driveRB.close();
        driveLF.close();
        driveLB.close();

    }
}
