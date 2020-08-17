package org.firstinspires.ftc.teamcode.MotorTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LinerOpModeMotorPractice extends LinearOpMode {

    private DcMotor motor;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "motor");

        waitForStart();

        while (opModeIsActive())
            motor.setPower(0.5f);

    }

    public static class LinearOpModeMotor extends LinearOpMode {
        private DcMotor motor;
        private ElapsedTime elapseTime = new ElapsedTime();

        @Override
        public void runOpMode() {
            motor = hardwareMap.get(DcMotor.class,"Motor");

            motor.setDirection(DcMotor.Direction.FORWARD);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            waitForStart();
            elapseTime.reset();

            while (opModeIsActive()) {
                motor.setPower(0.5f);
                telemetry.addData("Motor Running", elapseTime.time() );
                telemetry.update();
            }

            while ( elapseTime.time() < 5 ) {
                motor.setPower(0.5f);
                telemetry.addData("Motor Running", elapseTime.time() );
                telemetry.update();
            }
        }
    }
}
