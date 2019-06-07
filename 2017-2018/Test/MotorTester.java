package org.firstinspires.ftc.teamcode.Test;

import com.revAmped.components.Button;
import com.revAmped.components.HwLed;
import com.revAmped.components.HwMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import java.text.DecimalFormat;

/**
 * Created by Parthiv Nair on 1/30/2018.
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ParthivTestMotor", group = "Individual")
public class MotorTester extends OpMode {
    public HwMotor motor;
    private final static DecimalFormat numberFormatter = new DecimalFormat("######");
    public HwLed ledBlue;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        hardwareMap.logDevices();

        RobotLog.e("Initializing motors");
        //Initialize Motors
        try {
            motor = new HwMotor(hardwareMap,
                    "motor",
                    DcMotor.Direction.FORWARD);
            motor.resetPosition();
            motor.setPower(0f);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            ledBlue = new HwLed(hardwareMap, "led_blue");
            ledBlue.off();
            telemetry.addData("Status", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Status", "missing: motor " + e.getMessage());
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        float x1 = gamepad1.left_stick_x;
        x1 = Range.clip(x1, -1f, 1f);
        if (x1 < 0) {
            ledBlue.on();
        } else {
            ledBlue.off();
        }
        motor.setPower(x1);
        telemetry.addData("Encoder", numberFormatter.format(motor.getCurrentPosition()));
        telemetry.addData("Power", numberFormatter.format(x1));
        telemetry.addData("Stop", "Back");
        long timeStamp = System.currentTimeMillis();
        if (gamepad1.left_stick_button && Button.BTN_BACK.canPress(timeStamp)) {
            telemetry.addData("Stop", "True");
            stop();
        }
        telemetry.update();
    }
}
