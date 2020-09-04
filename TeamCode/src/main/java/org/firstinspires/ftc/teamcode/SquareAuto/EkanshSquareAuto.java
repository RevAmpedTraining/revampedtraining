package org.firstinspires.ftc.teamcode.SquareAuto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.revAmped.components.MecanumDrive;
import com.revAmped.components.RobotEncoderTest;

public class EkanshSquareAuto extends LinearOpMode {

    private DcMotor drivelf;
    private DcMotor drivelb;
    private DcMotor driverf;
    private DcMotor driverb;

    private RobotEncoderTest robot;

    @Override
    public void runOpMode() throws InterruptedException {
        drivelf = hardwareMap.get(DcMotor.class, "lf");
        drivelb = hardwareMap.get(DcMotor.class, "lb");
        driverf = hardwareMap.get(DcMotor.class, "rf");
        driverb = hardwareMap.get(DcMotor.class, "rb");

        drivelf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drivelb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driverf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driverb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        driveWithEncoder(0.6f, 10);
    }

    public void driveStraight(double power){
        drivelf.setPower(power);
        drivelb.setPower(power);
        driverf.setPower(power);
        driverb.setPower(power);
    }

    public void driveSideways(double power){
        drivelf.setPower(power);
        drivelb.setPower(-power);
        driverf.setPower(-power);
        driverb.setPower(power);
    }

    public void stopDriving(){
        driveStraight(0);
        driveSideways(0);
    }

    public void driveWithEncoder(double power, int distance){
        drivelf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drivelb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driverf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driverb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drivelf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drivelb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driverf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        driverb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drivelf.setTargetPosition(distance);
        drivelb.setTargetPosition(distance);
        driverf.setTargetPosition(distance);
        driverb.setTargetPosition(distance);


        driveStraight(power); //go forward
        driveSideways(1.4 * power);//go right bigger number bc of mecanum strafe
        driveStraight(-power);//go back
        driveSideways(-1.4 * power);//go left bigger number bc of mecanum strafe

        while(drivelf.isBusy() && drivelb.isBusy() && driverf.isBusy() && driverb.isBusy()){

        }

        stopDriving();

    }

}


