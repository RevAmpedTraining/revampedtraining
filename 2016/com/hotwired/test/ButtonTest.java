package com.revAmped.test;

import com.revAmped.components.Button;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by zwang on 2/17/2016.
 */
public class ButtonTest extends OpMode
{
    private float f = 0f;
    private Button BTN_X = new Button();

    public void init() {
    }

    public void loop() {
        long timestamp = System.currentTimeMillis();

        float x2 = gamepad1.right_stick_x;

        if(Math.abs(x2) >= 0.15 && BTN_X.canPress6Short(timestamp)) {
            f -= 0.015f * x2;
        }

        this.telemetry.addData("Back", this.gamepad1.back);
        this.telemetry.addData("LBumper", this.gamepad1.left_bumper);
        this.telemetry.addData("RTrigger", this.gamepad1.right_trigger);
        this.telemetry.addData("X", Float.toString(f));
        this.telemetry.addData("X2", Float.toString(x2));
        this.telemetry.addData("T", Long.toString(timestamp));

    }
}