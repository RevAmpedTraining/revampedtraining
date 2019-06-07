/*
Modern Robotics Range Sensor Example
Created 9/8/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.x Beta
Reuse permitted with credit where credit is due

Configuration:
I2cDevice on an Interface Module named "range" at the default address of 0x28 (0x14 7-bit)

This program can be run without a battery and Power Destitution Module.

For more information, visit modernroboticsedu.com.
Support is available by emailing support@modernroboticsinc.com.
*/

package org.firstinspires.ftc.teamcode.LastYear.RevAmpedTest;

import com.revAmped.linear.components.RobotRevAmpedLinear2;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


//@Disabled
@TeleOp(name = "Sonar Test", group = "Test")

public class SonarTestRevAmped extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private RobotRevAmpedLinear2 robot = null;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotRevAmpedLinear2(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData("PressToStart", "");
        telemetry.update();
        waitForStart();
        runtime.reset();
        double distanceL;
        double distanceR;
        while (opModeIsActive()) {
            distanceL = robot.sonarL.getDistance();
            distanceR = robot.sonarR.getDistance();
            telemetry.addData("Distance Left", distanceL);
            telemetry.addData("Distance Right", distanceR);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            idle();
        }
    }
}