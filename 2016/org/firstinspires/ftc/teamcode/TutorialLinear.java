package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by zwang on 6/11/2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="TutorialLinear", group="Test")
public class TutorialLinear extends LinearOpMode {

    public DcMotor newDcMotor = null;
    public Servo newServo = null;
    public DeviceInterfaceModule dim = null;

    @Override
    public void runOpMode() throws InterruptedException {

        // initialization
        newDcMotor = hardwareMap.dcMotor.get("dc_motor");
        newServo = hardwareMap.servo.get("servo");
        dim = hardwareMap.deviceInterfaceModule.get("Device Interface Module 2");

        newDcMotor.setDirection(DcMotor.Direction.FORWARD);
        newDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        newDcMotor.setPower(0);

        newServo.setPosition(0);
        dim.setDigitalChannelMode(0, DigitalChannel.Mode.OUTPUT);
        dim.setDigitalChannelState(0, false);

        waitForStart();

        // looping for actions
        long startTime = System.currentTimeMillis();
        long timeStamp = startTime;
        while (timeStamp - startTime < 5000) {
            newDcMotor.setPower(0.5f);
            newServo.setPosition(1f);
            dim.setDigitalChannelState(0, true);
            int encoderValue = newDcMotor.getCurrentPosition();
            telemetry.addData("Encoder", Integer.toString(encoderValue));
            telemetry.update();

            timeStamp = System.currentTimeMillis();
        }

        // clean up
        newDcMotor.setPower(0.5f);
        newServo.setPosition(1f);
        dim.setDigitalChannelState(0, true);
        int encoderValue = newDcMotor.getCurrentPosition();
        telemetry.addData("Encoder", Integer.toString(encoderValue));
        telemetry.update();
    }
}
