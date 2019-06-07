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

import com.revAmped.components.HwSonarAnalog;
import com.revAmped.linear.components.RobotRevAmpedLinear2;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;


//@Disabled
@TeleOp(name = "Sonar Optical Test", group = "Test")

public class OpticalSonarCombTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    //private RobotRevAmpedLinear2 robot = null;
    public HwSonarAnalog sonarTest;
    public HwSonarAnalog opticalTest;


    @Override
    public void runOpMode() throws InterruptedException {
        //robot = new RobotRevAmpedLinear2(this);
        sonarTest = new HwSonarAnalog(hardwareMap,
                "sonar_test",
                HwSonarAnalog.SCALE_MAX_XL);
        opticalTest = new HwSonarAnalog(hardwareMap,
                "optical_test",
                HwSonarAnalog.SCALE_MAX_XL);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.addData("PressToStart", "");
        telemetry.update();
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            float distance= sonarTest.getDistance();
            double distance1 = opticalTest.getOpticalDistance();
            telemetry.addData("Analog 1", distance);
            telemetry.addData("Analog 0", distance1);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            idle();
        }
    }
}
