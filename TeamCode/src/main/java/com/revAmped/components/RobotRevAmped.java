package com.revAmped.components;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;
import com.revAmped.config.RobotRevAmpedConstants;
import com.revAmped.config.SwerveDriveConstants;
import com.revAmped.sensors.RevColorDistanceSensor;
import com.revAmped.sensors.VuMarkSensing;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * Robot Hardware Initialization
 */
public class RobotRevAmped {
    protected Telemetry telemetry;
    public final List<HwNormalizedColorSensor> colorSensors = new ArrayList<>();
    //Drive Motors
    public final HwMotor driveLeftFront;
    public final HwMotor driveLeftBack;
    public final HwMotor driveRightFront;
    public final HwMotor driveRightBack;
    //Drive Servos
    public final HwServo servoLeftFront;
    public final HwServo servoLeftBack;
    public final HwServo servoRightFront;
    public final HwServo servoRightBack;
    //Color Sensors
   public  RevColorDistanceSensor revColorDistanceSensor;
    //Vuforia
    public  VuMarkSensing relicRecoveryVuMark;
    //Sonar Sensors
    public  List<HwDistanceSensor> sonars = new ArrayList<>();
    public  HwSonarAnalog sonarRF;
    public  List<HwSonarAnalog> sonarAnalogs = new ArrayList<>();
    //Gyro sensors
    public  HwGyro gyroSensor;
    //Range Sensors
    public  ModernRoboticsI2cRangeSensor rangeStick;
    //Non-drive motors
    public final HwMotor motorSlide;
    public final HwMotor motorClaw;
    public final HwMotor motorIntake;
    public final HwMotor motorTelescope;
    //================= Servos to be used
    public HwServo servoDumper;
    public HwServo servoMarkerDumper;
    public HwServo servoClaw;

    public  List<HwServo> servos = new ArrayList<>();
    //Switches
    public  HwSwitch switchSlideUp;
    public  HwSwitch switchSlideDown;
    public  List<HwSwitch> switchs = new ArrayList<>();
    //LEDs
    public  HwLed ledYellow;
    public  HwLed ledGreen;
    public  HwLed ledWhite;
    public  HwLed ledBlue;
    public  HwLed ledRed;
    public  List<HwLed> leds = new ArrayList<>();
    //Drive object
    public  Drive drive;

    /**
     * initialize the robot
     *
     * @param op           opMode to run
     * @param isAutonomous if autonomous
     */
    public RobotRevAmped(OpMode op,
                          boolean isAutonomous) {
        int numberMissing = 0;
        HardwareMap hardwareMap = op.hardwareMap;
        this.telemetry = op.telemetry;

        hardwareMap.logDevices();
        //Initialize Motors
        HwMotor driveLeftFront = null;
        try {
            driveLeftFront = new HwMotor(hardwareMap,
                    "driveLF",
                    DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            RobotLog.e("missing: driveLF " + e.getMessage());
            numberMissing++;
        }
        this.driveLeftFront = driveLeftFront;

        HwMotor driveLeftBack = null;
        try {
            driveLeftBack = new HwMotor(hardwareMap,
                    "driveLB",
                    DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            RobotLog.e("missing: driveLB " + e.getMessage());
            numberMissing++;
        }
        this.driveLeftBack = driveLeftBack;

        HwMotor driveRightFront = null;
        try {
            driveRightFront = new HwMotor(hardwareMap,
                    "driveRF",
                    DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            RobotLog.e("missing: driveRF " + e.getMessage());
            numberMissing++;
        }
        this.driveRightFront = driveRightFront;

        HwMotor driveRightBack = null;
        try {
            driveRightBack = new HwMotor(hardwareMap,
                    "driveRB",
                    DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            RobotLog.e("missing: driveRB" + e.getMessage());
            numberMissing++;
        }
        this.driveRightBack = driveRightBack;
        //drive servos
        HwServo servoLeftFront = null;
        try{
            servoLeftFront = new HwServo(hardwareMap,
                    "servoLF",
                    SwerveDriveConstants.SERVO_LEFTFRONT_START);
        } catch (Exception e) {
            RobotLog.e("missing: servoLF" + e.getMessage());
        }
        this.servoLeftFront = servoLeftFront;

        HwServo servoLeftBack = null;
        try{
            servoLeftBack = new HwServo(hardwareMap,
                    "servoLB",
                    SwerveDriveConstants.SERVO_LEFTBACK_START);
        } catch (Exception e) {
            RobotLog.e("missing: servoLB" + e.getMessage());
        }
        this.servoLeftBack = servoLeftBack;

        HwServo servoRightFront = null;
        try{
            servoRightFront = new HwServo(hardwareMap,
                    "servoRF",
                    SwerveDriveConstants.SERVO_RIGHTFRONT_START);
        } catch (Exception e) {
            RobotLog.e("missing: servoRF" + e.getMessage());
        }
        this.servoRightFront = servoRightFront;

        HwServo servoRightBack = null;
        try{
            servoRightBack = new HwServo(hardwareMap,
                    "servoRB",
                    SwerveDriveConstants.SERVO_RIGHTBACK_START);
        } catch (Exception e) {
            RobotLog.e("missing: servoRB" + e.getMessage());
        }
        this.servoRightBack = servoRightBack;

        HwMotor motorSlide = null;
        try {
            motorSlide = new HwMotor(hardwareMap,
                    "slideMotor",
                    DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            RobotLog.e("missing: motor_slide " + e.getMessage());
            numberMissing++;
        }
        this.motorSlide = motorSlide;

        HwMotor motorClaw = null;
        try {
            motorClaw = new HwMotor(hardwareMap,
                    "clawMotor",
                   DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            RobotLog.e("missing: motor_claw" + e.getMessage());
        }
        this.motorClaw = motorClaw;

        HwMotor motorIntake = null;
        try {
            motorIntake = new HwMotor(hardwareMap,
                    "intakeMotor",
                    DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            RobotLog.e("missing: motor_intake" + e.getMessage());
        }
        this.motorIntake = motorIntake;

        HwMotor motorTelescope = null;
        try{
            motorTelescope = new HwMotor(hardwareMap,
                    "telescopingMotor",
                    DcMotor.Direction.FORWARD
                            );
        } catch (Exception e) {
            RobotLog.e("missing: telescoping_motor" + e.getMessage());
        }
        this.motorTelescope = motorTelescope;

        //Initialize servos used on the robot
        HwServo servoDumper = null;
        try {
            servoDumper = new HwServo(hardwareMap,
                    "servoDumper",
                    0);
            servos.add(servoDumper);
        } catch (Exception e) {
            RobotLog.e("missing: servo_dumper " + e.getMessage());
            numberMissing++;
        }
        this.servoDumper = servoDumper;

        HwServo servoMarkerDumper = null;
        try{
            servoMarkerDumper = new HwServo(
                    hardwareMap,
                    "servoMarkerDumper",
                    0
            );

        }catch (Exception e) {
            RobotLog.e("missing: servo_markerdumper" + e.getMessage());
        }
        this.servoMarkerDumper = servoMarkerDumper;

        HwServo servoClaw = null;
        try{
            servoClaw = new HwServo(
                    hardwareMap,
                    "servoClaw",
                    0
            );

        }catch (Exception e) {
            RobotLog.e("missing: servo_claw" + e.getMessage());
        }
        this.servoClaw = servoClaw;


        //Initialize Switch Sensors
        HwSwitch switchSlideup = null;
        try {
            switchSlideup = new HwSwitch(hardwareMap,
                    "switch_slideup",
                    true);
            switchs.add(switchSlideup);
        } catch (Exception e) {
            RobotLog.e("missing: digital channel " + e.getMessage());
            numberMissing++;
        }
        this.switchSlideUp = switchSlideup;
        //Initializing LED panel
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
        //Initialize I2C Sensors
        RevColorDistanceSensor revColorDistanceSensor = null;
        if(isAutonomous) {
            try {
                revColorDistanceSensor = new RevColorDistanceSensor(op.hardwareMap, "jewel_color");
            } catch (Exception e) {
                RobotLog.e("missing: jewel_color_sensor " + e.getMessage());
                numberMissing++;
            }
        }

        this.revColorDistanceSensor = revColorDistanceSensor;

        ModernRoboticsI2cRangeSensor rangeStick = null;
        if (isAutonomous) {
            try {
                rangeStick = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_stick");
            } catch (Exception e) {
                RobotLog.e("missing: range_stick" + e.getMessage());
            }
        }
        this.rangeStick= rangeStick;

        //Initialize the Sonar sensors used
       sonarRF = new HwSonarAnalog(hardwareMap,
                "sonar_rf",
                HwSonarAnalog.SCALE_MAX_XL);
        sonarAnalogs.add(sonarRF);

        VuMarkSensing relicRecoveryVuMark = null;
        if (isAutonomous) {
            try {
                relicRecoveryVuMark = new VuMarkSensing(op.hardwareMap);
                Boolean bInit = relicRecoveryVuMark.initialize();
                if(bInit)
                {
                    RobotLog.vv("OpenCV","Initialized Successfully");
                }
                else
                {
                    RobotLog.vv("OpenCV", "Initialization might be failed.");
                }
            } catch (Exception e) {
                RobotLog.e("missing: VuMark Sensing" + e.getMessage());
                numberMissing++;
            }
        }
        this.relicRecoveryVuMark = relicRecoveryVuMark;
        //Initialize GyroSensor
        HwGyro gyroSensor = null;
        if (isAutonomous) {
            try {
                gyroSensor = new HwBnoGyro(hardwareMap,
                        "imu");
                while (isAutonomous && gyroSensor.isCalibrating()) {
                    telemetry.addData("Gyro", "Calibrating");
                    telemetry.update();
                    Thread.sleep(50);
                }
            } catch (Exception e) {
                RobotLog.e("missing: gyro_sensor imu" + e.getMessage());
                numberMissing++;
            }
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

    }

    /**
     * subclass override this method
     *
     * @return Drive
     */
    protected Drive createDrive() {
        return new TankDrive(driveLeftFront,
                driveLeftBack,
                driveRightFront,
                driveRightBack);
    }

    /**
     * initialize tank drive
     *
     * @return TankDrive
     */
    public TankDrive getTankDrive() {
        return (TankDrive) this.drive;
    }

    /**
     * initialize swerve drive
     *
     * @return SwerveDrive
     */
    public SwerveDrive getSwerveDrive() {return (SwerveDrive) this.drive;}

    /**
     * update LEDs
     */
    public void drawLed() {
        for (HwLed led : leds) {
            led.draw();
        }
    }

}
