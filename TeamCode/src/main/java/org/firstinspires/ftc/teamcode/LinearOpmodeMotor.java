package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LinearOpmodeMotor extends LinearOpMode {

    private DcMotor motor;
    private ElapsedTime elapsedTime;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, "motor_demo");
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        elapsedTime.reset();

        while(opModeIsActive()){
            motor.setPower(0.5f);

            telemetry.addData("Motor running", elapsedTime);


        }

    }


}
