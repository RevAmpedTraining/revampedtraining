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
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * Robot Hardware Initialization
 */
public class RobotRevampedTest {
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
    // public  RevColorDistanceSensor revColorDistanceSensor;
    //Vuforia
    public  VuMarkSensing relicRecoveryVuMark;
    //Sonar Sensors
    public  List<HwDistanceSensor> sonars = new ArrayList<>();
    //public  HwSonarAnalog sonarRF;
    public  List<HwSonarAnalog> sonarAnalogs = new ArrayList<>();
    //Gyro sensors
    public  HwGyro gyroSensor;
    //Range Sensors
    //public  ModernRoboticsI2cRangeSensor rangeStick;
    //Non-drive motors
    public final HwMotor motorLatch;
    public final HwMotor motorSlide;
    public final HwMotor motorIntake;
    public final HwMotor motorPopper;
    public final CRServo servoTelescopeL;
    public final CRServo servoTelescopeR;
    //================= Servos to be used
    //public HwServo servoLatch;
    public HwServo servoTeam;
    public HwServo servoDrop;
    public HwServo servoHold;
    public HwServo servoDump;
    public HwServo servoLatch;

    public  List<HwServo> servos = new ArrayList<>();
    //Switches
    public  HwSwitch switchSlideUp;
    public  HwSwitch switchSlideIn;
    public HwSwitch switchSlideDown;
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
    public RobotRevampedTest(OpMode op,
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
            servos.add(servoLeftFront);
        } catch (Exception e) {
            RobotLog.e("missing: servoLF" + e.getMessage());
        }
        this.servoLeftFront = servoLeftFront;

        HwServo servoLeftBack = null;
        try{
            servoLeftBack = new HwServo(hardwareMap,
                    "servoLB",
                    SwerveDriveConstants.SERVO_LEFTBACK_START);
            servos.add(servoLeftBack);
        } catch (Exception e) {
            RobotLog.e("missing: servoLB" + e.getMessage());
        }
        this.servoLeftBack = servoLeftBack;

        HwServo servoRightFront = null;
        try{
            servoRightFront = new HwServo(hardwareMap,
                    "servoRF",
                    SwerveDriveConstants.SERVO_RIGHTFRONT_START);
            servos.add(servoRightFront);
        } catch (Exception e) {
            RobotLog.e("missing: servoRF" + e.getMessage());
        }
        this.servoRightFront = servoRightFront;

        HwServo servoRightBack = null;
        try{
            servoRightBack = new HwServo(hardwareMap,
                    "servoRB",
                    SwerveDriveConstants.SERVO_RIGHTBACK_START);
            servos.add(servoRightBack);
        } catch (Exception e) {
            RobotLog.e("missing: servoRB" + e.getMessage());
        }
        this.servoRightBack = servoRightBack;

        HwServo servoTeam = null;
        try{
            servoTeam = new HwServo(hardwareMap,
                    "servoTM",
                    RobotRevAmpedConstants.SERVO_MARKER_IN);
                    servos.add(servoTeam);
        } catch (Exception e){
            RobotLog.e("missing: servoTM" + e.getMessage());

            numberMissing++;
        }
        this.servoTeam = servoTeam;

        HwMotor motorLatch = null;
        try {
            motorLatch = new HwMotor(hardwareMap,
                    "latchMotor",
                    DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            RobotLog.e("missing: motor_latch " + e.getMessage());
            telemetry.addLine("missing: latch motor");
            telemetry.update();
            numberMissing++;
        }
        this.motorLatch = motorLatch;

        HwMotor motorIntake = null;
        try {
            motorIntake = new HwMotor(hardwareMap,
                    "motorIntake",
                   DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            RobotLog.e("missing: motor_intake" + e.getMessage());
            telemetry.addLine("missing: motor intake");
            telemetry.update();
        }
        this.motorIntake = motorIntake;

        HwMotor motorPopper = null;
        try {
            motorPopper = new HwMotor(hardwareMap,
                    "motorPopper",
                    DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            RobotLog.e("missing: motor_popper" + e.getMessage());
            telemetry.addLine("missing: motor popper");
            telemetry.update();
        }
        this.motorPopper = motorPopper;

        HwMotor motorSlide = null;
        try{
            motorSlide = new HwMotor(hardwareMap,
                    "motorSlide",
                    DcMotor.Direction.FORWARD
                            );
        } catch (Exception e) {
            RobotLog.e("missing: Slide_motor" + e.getMessage());
            telemetry.addLine("missing: motor slide");
            telemetry.update();
        }
        this.motorSlide = motorSlide;

        //Initialize servos used on the robot
       /* HwServo servoDrop = null;
        try {
            servoDrop = new HwServo(hardwareMap,
                    "servoDrop",
                    RobotRevAmpedConstants.INTAKE_DROP);
            servos.add(servoDrop);
        } catch (Exception e) {
            RobotLog.e("missing: servo_drop " + e.getMessage());
            numberMissing++;
        }
        this.servoDrop = servoDrop;

        HwServo servoHold = null;
        try{
            servoHold = new HwServo(
                    hardwareMap,
                    "servoHold",
                    RobotRevAmpedConstants.INTAKE_HOLD
            );
            servos.add(servoHold);
        }catch (Exception e) {
            RobotLog.e("missing: servo_hold" + e.getMessage());
        }
        this.servoHold = servoHold;*/

        CRServo servoTelescopeL = null;
        try{
            servoTelescopeL = hardwareMap.get(CRServo.class,
                    "servoTelescopeL");
        }catch (Exception e) {
            RobotLog.e("missing: servo_telescope" + e.getMessage());
        }
        this.servoTelescopeL = servoTelescopeL;

        CRServo servoTelescopeR = null;
        try{
            servoTelescopeR = hardwareMap.get(CRServo.class,
                    "servoTelescopeR");
        }catch (Exception e) {
            RobotLog.e("missing: servo_telescope" + e.getMessage());
            telemetry.addLine("missing: servo_telescope");
            telemetry.update();
        }
        this.servoTelescopeR = servoTelescopeR;

        HwServo servoDump = null;
        try{
            servoDump = new HwServo(
                    hardwareMap,
                    "servoDump",
                    //RobotRevAmpedConstants.SERVO_DUMP_INIT
                    56/255f
            );
            servos.add(servoDump);
        }catch (Exception e) {
            RobotLog.e("missing: servo_dump" + e.getMessage());
            telemetry.addLine("missing: servo_dump");
            telemetry.update();
        }
        this.servoDump = servoDump;

        HwServo servoLatch = null;
        try{
            servoLatch = new HwServo(
                    hardwareMap,
                    "servoLatch",
                    RobotRevAmpedConstants.SERVO_LATCH_OUT
            );
            servos.add(servoLatch);
        }catch (Exception e) {
            RobotLog.e("missing: servo_latch" + e.getMessage());
            telemetry.addLine("missing: servo_latch");
            telemetry.update();
        }
        this.servoLatch = servoLatch;


        //Initialize Switch Sensors
       HwSwitch switchSlideup = null;
        try {
            switchSlideup = new HwSwitch(hardwareMap,
                    "switch_slideup",
                    true);
            switchs.add(switchSlideup);
        } catch (Exception e) {
            telemetry.addLine("missing: switchup");
            telemetry.update();
            RobotLog.e("missing: digital channel " + e.getMessage());
            numberMissing++;
        }
        this.switchSlideUp = switchSlideup;

        HwSwitch switchSlidein = null;
        try {
            switchSlidein = new HwSwitch(hardwareMap,
                    "switch_slidein",
                    true);
            switchs.add(switchSlidein);
        } catch (Exception e) {
            telemetry.addLine("switchin");
            telemetry.update();
            RobotLog.e("missing: digital channel " + e.getMessage());
            numberMissing++;
        }
        this.switchSlideIn = switchSlidein;

        HwSwitch switchSlidedown = null;
        try {
            switchSlidedown = new HwSwitch(hardwareMap,
                    "switch_slidedown",
                    true);
            switchs.add(switchSlidedown);
        } catch (Exception e) {
            telemetry.addData("switch", "here");
            telemetry.update();
            RobotLog.e("missing: digital channel " + e.getMessage());
            numberMissing++;
        }
        this.switchSlideDown = switchSlidedown;
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
        /*
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
        this.relicRecoveryVuMark = relicRecoveryVuMark;*/
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
                telemetry.addData("imu", "here");
                telemetry.update();
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
    /*
    Robot motor shutdown
     */
    public void close1() {
        driveRightFront.setPower(0);
        driveRightBack.setPower(0);
        driveLeftBack.setPower(0);
        driveLeftFront.setPower(0);
        motorIntake.setPower(0);
        motorSlide.setPower(0);
        motorPopper.setPower(0);
        motorLatch.setPower(0);
        servoTelescopeL.setPower(0);
        servoTelescopeR.setPower(0);
    }
    /**
     * subclass override this method
     *
     * @return Drive
     */
    protected Drive createDrive() {
        return new SwerveDrive(driveLeftFront,
                driveLeftBack,
                driveRightFront,
                driveRightBack,
                servoLeftFront,
                servoLeftBack,
                servoRightFront,
                servoLeftBack);
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
