package org.firstinspires.ftc.teamcode.LastYear.RevAmpedTest;

/*
Modern Robotics Range Sensors Example
Created 10/31/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.35
Reuse permitted with credit where credit is due

Configuration:
I2CDevice "range28" (MRI Range Sensor with default I2C address 0x28
I2CDevice "range2a" (MRI Color Sensor with I2C address 0x2a

ModernRoboticsI2cGyro is not being used because it does not support .setI2CAddr().

To change range sensor I2C Addresses, go to http://modernroboticsedu.com/mod/lesson/view.php?id=96
Support is available by emailing support@modernroboticsinc.com.
*/


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Range Sensors2", group = "MRI")
//@Disabled
public class MRI_Range_Sensors extends OpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();


    ModernRoboticsI2cRangeSensor rangeStick;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        rangeStick = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_stick");

    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        float distStick = (float) rangeStick.getDistance(DistanceUnit.CM);

        telemetry.addData("RangeSensorLeft", "%f", distStick);

        RobotLog.aa("distStick=", "%f",  distStick);

        telemetry.update();

    }
}