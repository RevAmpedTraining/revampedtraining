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

import com.revAmped.config.RobotRevAmpedConstants;
import com.revAmped.linear.components.RobotRevAmpedLinear2;
import com.revAmped.sensors.RevColorDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.revAmped.util.GlyphDetector;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//@Disabled
@TeleOp(name = "GlyphDetectorTest2", group = "MRI")

public class GlyphDetectorTest extends LinearOpMode {

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

        float dist_door, dist_tray;
        int count = 0;
        double distance;
        GlyphDetector g_detector = new GlyphDetector(this.hardwareMap, "sensor_front", "sensor_back", "sonar_rf");
        RevColorDistanceSensor cryptobox = new RevColorDistanceSensor(this.hardwareMap, "jewel_color");
        while (opModeIsActive()) {
            //range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            robot.servoJewel.setPosition(RobotRevAmpedConstants.SERVO_CRYPTOBOX_NEAR);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            distance = cryptobox.GetDistance();
            dist_door =  g_detector.GetDistance("sensor_front");
            dist_tray =  g_detector.GetDistance("sensor_back");
            telemetry.addData("distance", distance);
            telemetry.addData("Door", "%4.1f", dist_door);
            telemetry.addData("Tray", "%4.1f", dist_tray);
            count = g_detector.GetGlyphCount(count);
            telemetry.addData("Count", count);
            telemetry.update();

            idle();
        }
    }


    private double getDistrance(DistanceUnit du)
    {
        double val = -1;
        switch(du) {
            case CM:
                //val = (ultrasonicRange.getVoltage() - .6050) / .0175 * 2.54;
                break;
            case INCH:
                //val = (ultrasonicRange.getVoltage() - .6050) / .0175;
            default:
                break;
        }
        return val;
    }
}