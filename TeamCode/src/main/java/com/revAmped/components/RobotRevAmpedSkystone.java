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

/**
 * Created by swang4 on 6/21/2019.
 */

public class RobotRevAmpedSkystone {

    protected Telemetry telemetry;

    public final HwMotor driveLeftFront;
    public final HwMotor driveLeftBack;
    public final HwMotor driveRightFront;
    public final HwMotor driveRightBack;
    public final HwMotor intakeL;
    public final HwMotor intakeR;
    public final HwMotor vertL;
    public final HwMotor vertR;

    public HwGyro gyroSensor;

    public AnalogInput x_encoder;
    public AnalogInput y_encoder;
    public AnalogInput y2_encoder;

    public HwSwitch switchSlideDown;
    public HwSwitch revSwitch;
    public  List<HwSwitch> switchs = new ArrayList<>();

    public HwServo dumpClaw;
    public HwServo turnClaw;
    public HwServo horSlide;
    public HwServo latchLeft;
    public HwServo latchRight;
    public HwServo servoCap;
    public HwServo stopper;
    public CRServo tapeMeasure;
    public CRServo tapeAuto;

    public final HwLed ledYellow;
    public final HwLed ledGreen;
    public final HwLed ledWhite;
    public final HwLed ledBlue;
    public final HwLed ledRed;
    public final List<HwLed> leds = new ArrayList<>();

    public List<HwServo> servos = new ArrayList<>();

    public final HwSonarAnalog sonarR;
    public final HwSonarAnalog sonarL;
    public final HwSonarAnalog sonarF;
    public final List<HwSonarAnalog> sonarAnalogs = new ArrayList<>();

    public DistanceSensor distance;

    public Drive drive;

    public RobotRevAmpedSkystone(OpMode op, boolean isAutonomous) {
        int numberMissing = 0;
        HardwareMap hardwareMap = op.hardwareMap;
        this.telemetry = op.telemetry;

        hardwareMap.logDevices();
        //Initialize Motors
        ledYellow = new HwLed(hardwareMap,
                "led_yellow");
        leds.add(ledYellow);
        ledGreen = new HwLed(hardwareMap,
                "led_green");
        leds.add(ledGreen);
        ledWhite = new HwLed(hardwareMap,
                "led_white");
        leds.add(ledWhite);
        ledBlue = new HwLed(hardwareMap,
                "led_blue");
        leds.add(ledBlue);
        ledRed = new HwLed(hardwareMap,
                "led_red");
        leds.add(ledRed);

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

        HwMotor intakeL = null;
        try {
            intakeL = new HwMotor(hardwareMap,
                    "intake_l",
                    DcMotorSimple.Direction.FORWARD);
        } catch (Exception e) {
            RobotLog.e("missing: intake_l" + e.getMessage());
            telemetry.addLine("missing: intake left");
            numberMissing++;
        }
        this.intakeL = intakeL;

        HwMotor intakeR = null;
        try {
            intakeR = new HwMotor(hardwareMap,
                    "intake_r",
                    DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            RobotLog.e("missing: intake_r" + e.getMessage());
            telemetry.addLine("missing: intake right");
            numberMissing++;
        }
        this.intakeR = intakeR;

        HwMotor vertL = null;
        try {
            vertL = new HwMotor(hardwareMap,
                    "vert_slide_l",
                    DcMotorSimple.Direction.FORWARD);
        } catch  (Exception e) {
            RobotLog.e("missing: vert_l" + e.getMessage());
            numberMissing++;
        }
        this.vertL = vertL;

        HwMotor vertR = null;
        try {
            vertR = new HwMotor(hardwareMap,
                    "vert_slide_r",
                    DcMotorSimple.Direction.FORWARD);
        } catch  (Exception e) {
            RobotLog.e("missing: vert_r" + e.getMessage());
            numberMissing++;
        }
        this.vertR = vertR;

        HwServo latchLeft = null;
        try {
            latchLeft = new HwServo(hardwareMap,
                    "latch_left",
                    RobotRevAmpedConstants.LATCH_INL);
            servos.add(latchLeft);
        } catch (Exception e) {
            RobotLog.e("missing: latch_left");
            telemetry.addLine("missing: latch left");
            numberMissing++;
        }
        this.latchLeft = latchLeft;

        HwServo latchRight = null;
        try {
            latchRight = new HwServo(hardwareMap,
                    "latch_right",
                    RobotRevAmpedConstants.LATCH_INR);
            servos.add(latchRight);
        } catch (Exception e) {
            RobotLog.e("missing: latch_right");
            telemetry.addLine("missing: latch right");
            numberMissing++;
        }
        this.latchRight = latchRight;

        HwServo dumpClaw = null;
        try {
            dumpClaw = new HwServo(hardwareMap,
                    "dump_claw",
                    RobotRevAmpedConstants.DUMP_START);
            servos.add(dumpClaw);
        } catch (Exception e) {
            RobotLog.e("missing: dump_claw");
            telemetry.addLine("missing: dump claw");
            numberMissing++;
        }
        this.dumpClaw = dumpClaw;

        HwServo turnClaw = null;
        try {
            turnClaw = new HwServo(hardwareMap,
                    "turn_claw",
                    RobotRevAmpedConstants.CLAW_MID);
            servos.add(turnClaw);
        } catch (Exception e) {
            RobotLog.e("missing: turn_claw");
            telemetry.addLine("missing: turn claw");
            numberMissing++;
        }
        this.turnClaw = turnClaw;

        HwServo horSlide = null;
        try {
            horSlide = new HwServo(hardwareMap,
                    "hor_slide",
                    RobotRevAmpedConstants.H_IN);
            servos.add(horSlide);
        } catch (Exception e) {
            RobotLog.e("missing: hor_slide");
            numberMissing++;
        }
        this.horSlide = horSlide;

        HwServo servoCap = null;
        try{
            servoCap = new HwServo(hardwareMap,
                        "servo_cap",
                        RobotRevAmpedConstants.CAP_START);
            servos.add(servoCap);
        } catch (Exception e) {
            RobotLog.e("missing: capstone servo");
            numberMissing++;
        }
        this.servoCap = servoCap;

        HwServo stopper = null;
        try {
            stopper = new HwServo(hardwareMap,
                    "stopper",
                    RobotRevAmpedConstants.STOPPER);
            servos.add(stopper);
        } catch (Exception e) {
            RobotLog.e("missing: stopper servo");
            numberMissing++;
        }
        this.stopper = stopper;

        CRServo tapeMeasure = null;
        try{
            tapeMeasure = hardwareMap.get(CRServo.class,
                    "tape_measure");
        }catch (Exception e) {
            RobotLog.e("missing: tape measure" + e.getMessage());
        }
        this.tapeMeasure = tapeMeasure;

        CRServo tapeAuto = null;
        try{
            tapeAuto = hardwareMap.get(CRServo.class,
                    "tape_measure_auto");
        }catch (Exception e) {
            RobotLog.e("missing: tape auto" + e.getMessage());
        }
        this.tapeAuto = tapeAuto;

        HwGyro gyroSensor = null;
            try {
                gyroSensor = new HwBnoGyro(hardwareMap,
                        "imu");
                while (isAutonomous && gyroSensor.isCalibrating()) {
                    telemetry.addData("Gyro", "Calibrating");
                    telemetry.update();
                    Thread.sleep(50);
                }
            } catch (Exception e) {
                telemetry.addData("imu", "here");
                telemetry.update();
                RobotLog.e("missing: gyro_sensor imu" + e.getMessage());
                numberMissing++;
            }
        this.gyroSensor = gyroSensor;

        AnalogInput xEncoder = null;
        try {
            xEncoder = hardwareMap.get(AnalogInput.class,
                    "x_encoder");
        } catch (Exception e) {
            RobotLog.e("missing: x encoder" + e.getMessage());
            telemetry.addLine("missing: x encoder");
            numberMissing++;
        }
        this.x_encoder = xEncoder;

        AnalogInput yEncoder = null;
        try {
            yEncoder = hardwareMap.get(AnalogInput.class,
                    "y_encoder");
        } catch (Exception e) {
            RobotLog.e("missing: y encoder" + e.getMessage());
            telemetry.addLine("missing: y encoder");
            numberMissing++;
        }
        this.y_encoder = yEncoder;

        AnalogInput y2Encoder = null;
        try {
            y2Encoder = hardwareMap.get(AnalogInput.class,
                    "y2_encoder");
        } catch (Exception e) {
            RobotLog.e("missing: y2 encoder" + e.getMessage());
            telemetry.addLine("missing: y2 encoder");
            numberMissing++;
        }
        this.y2_encoder = y2Encoder;


        sonarR = new HwSonarAnalog(hardwareMap,
                    "sonar_r",
                    HwSonarAnalog.SCALE_MAX_XL);
        sonarAnalogs.add(sonarR);

        sonarL = new HwSonarAnalog(hardwareMap,
                "sonar_l",
                HwSonarAnalog.SCALE_MAX_XL);
        sonarAnalogs.add(sonarL);

        sonarF = new HwSonarAnalog(hardwareMap,
                "sonar_f",
                HwSonarAnalog.SCALE_MAX_XL);
        sonarAnalogs.add(sonarF);

        HwSwitch switchSlideDown = null;
        try {
            switchSlideDown = new HwSwitch(hardwareMap,
                    "switch_slide_down",
                    true);
            switchs.add(switchSlideDown);
        } catch (Exception e) {
            RobotLog.e("missing: digital channel " + e.getMessage());
            numberMissing++;
        }
        this.switchSlideDown = switchSlideDown;

        HwSwitch slideSwitch = null;
        try {
            slideSwitch = new HwSwitch(hardwareMap,
                    "slide_switch",
                    true);
            switchs.add(slideSwitch);
        } catch (Exception e) {
            RobotLog.e("missing: Rev Switch" + e.getMessage());
            numberMissing++;
        }
        this.revSwitch = slideSwitch;

        DistanceSensor distance = null;
        try {
            distance = hardwareMap.get(DistanceSensor.class, "distance");
        } catch (Exception e) {
            RobotLog.e("missing: Rev Distance" + e.getMessage());
            telemetry.addData("Missing", "Rev Distance");
            numberMissing++;
        }
        this.distance = distance;


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

    /**
     * update LEDs
     */
    public void drawLed() {
        for (HwLed led : leds) {
            led.draw();
        }
    }

}
