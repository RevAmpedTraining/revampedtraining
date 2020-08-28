package com.revAmped.components;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.RobotLog;
import com.revAmped.config.RobotRevAmpedConstants;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class RobotEncoderTest {

    protected Telemetry telemetry;

    public final HwMotor driveLeftFront;
    public final HwMotor driveLeftBack;
    public final HwMotor driveRightFront;
    public final HwMotor driveRightBack;
    public final HwMotor encoderYL;
    public final HwMotor encoderYR;
    public final HwMotor encoderX;

    public HwGyro gyroSensor;

    public Drive drive;

    public RobotEncoderTest(OpMode op, boolean isAutonomous) {
        int numberMissing = 0;
        HardwareMap hardwareMap = op.hardwareMap;
        this.telemetry = op.telemetry;

        hardwareMap.logDevices();
        //Initialize Motors

        HwMotor driveLeftFront = null;
        try {
            driveLeftFront = new HwMotor(hardwareMap,
                    "motor_lf",
                    DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            RobotLog.e("missing: motor_lf " + e.getMessage());
            telemetry.addLine("missing: motor lf");
            numberMissing++;
        }
        this.driveLeftFront = driveLeftFront;

        HwMotor driveLeftBack = null;
        try {
            driveLeftBack = new HwMotor(hardwareMap,
                    "motor_lb",
                    DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            RobotLog.e("missing: motor_lb " + e.getMessage());
            telemetry.addLine("missing: motor lb");
            numberMissing++;
        }
        this.driveLeftBack = driveLeftBack;

        HwMotor driveRightFront = null;
        try {
            driveRightFront = new HwMotor(hardwareMap,
                    "motor_rf",
                    DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            RobotLog.e("missing: motor_rf " + e.getMessage());
            telemetry.addLine("missing: motor rf");
            numberMissing++;
        }
        this.driveRightFront = driveRightFront;

        HwMotor driveRightBack = null;
        try {
            driveRightBack = new HwMotor(hardwareMap,
                    "motor_rb",
                    DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            RobotLog.e("missing: motor_rb" + e.getMessage());
            telemetry.addLine("missing: motor rb");
            numberMissing++;
        }
        this.driveRightBack = driveRightBack;

        HwMotor encoderYL = null;
        try {
            encoderYL = new HwMotor(hardwareMap,
                    "encoder_yl",
                    DcMotorSimple.Direction.FORWARD);
        } catch (Exception e) {
            RobotLog.e("missing: encoder_yl" + e.getMessage());
            numberMissing++;
        }
        this.encoderYL = encoderYL;

        HwMotor encoderYR = null;
        try {
            encoderYR = new HwMotor(hardwareMap,
                    "encoder_yr",
                    DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            RobotLog.e("missing: encoder_yr" + e.getMessage());
            telemetry.addLine("missing: intake right");
            numberMissing++;
        }
        this.encoderYR = encoderYR;

        HwMotor encoderX = null;
        try {
            encoderX = new HwMotor(hardwareMap,
                    "encoder_x",
                    DcMotorSimple.Direction.REVERSE);
        } catch  (Exception e) {
            RobotLog.e("missing: encoder_x" + e.getMessage());
            numberMissing++;
        }
        this.encoderX = encoderX;
        //navx
        /*HwGyro gyroSensor = null;
        try {
            gyroSensor = new HwNavGyro(hardwareMap,
                    "navx");
            while (isAutonomous && gyroSensor.isCalibrating()) {
                telemetry.addData("Gyro", "Calibrating");
                //telemetry.update();
                Thread.sleep(50);
            }
        } catch (Exception e) {
            telemetry.addData("navx", "here");
            RobotLog.e("missing: gyro_sensor imu" + e.getMessage());
            numberMissing++;
        }
        this.gyroSensor = gyroSensor;*/
        //bno
        HwGyro gyroSensor = null;
        try {
            gyroSensor = new HwBnoGyro(hardwareMap,
                    "imu");
            while (isAutonomous && gyroSensor.isCalibrating()) {
                telemetry.addData("Gyro", "Calibrating");
                //telemetry.update();
                Thread.sleep(50);
            }
        } catch (Exception e) {
            telemetry.addData("imu", "here");
            RobotLog.e("missing: gyro_sensor imu" + e.getMessage());
            numberMissing++;
        }
        this.gyroSensor = gyroSensor;

        this.drive = createDrive();

        telemetry.addData("Missing Devices", numberMissing);
        telemetry.update();


    }

    /**
     * Robot and sensor shut down
     */
    public void close() {
        RobotLog.vv("Close", "Stopped");
    }
    /**
     * subclass override this method
     *
     * @return Drive
     */
    protected Drive createDrive() {
        return new MecanumDrive(driveLeftFront,
                driveLeftBack,
                driveRightFront,
                driveRightBack);
    }


    /**
     * initialize swerve drive
     *
     * @return MecanumDrive
     */
    public MecanumDrive getMecanumDrive() {return (MecanumDrive) this.drive;}

}
