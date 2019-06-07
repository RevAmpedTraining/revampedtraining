package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Tutorial
 */
@TeleOp(name="Tutorial", group="Test")
public class Tutorial
        extends OpMode {


    public DcMotor newDcMotor = null;
    public Servo newServo = null;
    public DeviceInterfaceModule dim = null;

    @Override
    public void init() {
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
    }

    @Override
    public void stop() {
        newDcMotor.setPower(0);
        dim.setDigitalChannelState(0, false);
        newServo.setPosition(0.5f);
    }

    @Override
    public void loop() {
        newDcMotor.setPower(0.5f);
        newServo.setPosition(1f);
        dim.setDigitalChannelState(0, true);
        int encoderValue = newDcMotor.getCurrentPosition();
        telemetry.addData("Encoder", Integer.toString(encoderValue));
        telemetry.update();
    }
}
