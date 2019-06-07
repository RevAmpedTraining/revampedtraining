package com.revAmped.components;

import com.qualcomm.robotcore.robot.Robot;
import com.revAmped.config.RobotRevAmpedConstants;
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
    public  HwMotor motorSlide;

    //================= Servos to be used
    public  HwServo servoContainer;
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
                    "motorLF",
                    DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            RobotLog.e("missing: motorLF " + e.getMessage());
            numberMissing++;
        }
        this.driveLeftFront = driveLeftFront;

        HwMotor driveLeftBack = null;
        try {
            driveLeftBack = new HwMotor(hardwareMap,
                    "motorLB",
                    DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            RobotLog.e("missing: motorLB " + e.getMessage());
            numberMissing++;
        }
        this.driveLeftBack = driveLeftBack;

        HwMotor driveRightFront = null;
        try {
            driveRightFront = new HwMotor(hardwareMap,
                    "motorRF",
                    DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            RobotLog.e("missing: motorRF " + e.getMessage());
            numberMissing++;
        }
        this.driveRightFront = driveRightFront;

        HwMotor driveRightBack = null;
        try {
            driveRightBack = new HwMotor(hardwareMap,
                    "motorRB",
                    DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            RobotLog.e("missing: motorRB" + e.getMessage());
            numberMissing++;
        }
        this.driveRightBack = driveRightBack;

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

        //Initialize servos used on the robot
        HwServo servoContainer = null;
        try {
            servoContainer = new HwServo(hardwareMap,
                    "servo_cr",
                    RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
            servos.add(servoContainer);
        } catch (Exception e) {
            RobotLog.e("missing: servo_cr " + e.getMessage());
            numberMissing++;
        }
        this.servoContainer = servoContainer;
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
                gyroSensor = new HwNavGyro(hardwareMap,
                        "imu3");
                while (isAutonomous && gyroSensor.isCalibrating()) {
                    telemetry.addData("Gyro", "Calibrating");
                    telemetry.update();
                    Thread.sleep(50);
                }
            } catch (Exception e) {
                RobotLog.e("missing: gyro_sensor imu3" + e.getMessage());
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
     * update LEDs
     */
    public void drawLed() {
        for (HwLed led : leds) {
            led.draw();
        }
    }

}
