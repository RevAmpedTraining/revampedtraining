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
 * Robot definition for teleop
 */
public class RobotRevAmped2 {
    protected Telemetry telemetry;
    public final HwNormalizedColorSensor jewelColorSensor = null;
    public final List<HwNormalizedColorSensor> colorSensors = new ArrayList<>();
    public final RevColorDistanceSensor revColorDistanceSensor;
    public final VuMarkSensing relicRecoveryVuMark;
    public final List<HwDistanceSensor> sonars = new ArrayList<>();
    ///Sonar Sensors
    public final HwSonarAnalog sonarRF;
    public final HwSonarAnalog sonarL;
    public final HwSonarAnalog sonarR;
    //public final HwSonarAnalog sonarTest;
    public final List<HwSonarAnalog> sonarAnalogs = new ArrayList<>();

    public final HwGyro gyroSensor;

    public final ModernRoboticsI2cRangeSensor rangeStick;

    public final HwMotor driveLeftFront;
    public final HwMotor driveLeftBack;
    public final HwMotor driveRightFront;
    public final HwMotor driveRightBack;
    public final HwMotor sweeperLeft;
    public final HwMotor sweeperRight;
    public final HwMotor motorSlide;
    public final HwMotor motorRelicSlide;

    //================= Servos to be used
    public final HwServo servoContainer;
    public final HwServo servoDoorRight;
    public final HwServo servoJewel;
    public final HwServo servoJewelHit;
    public final HwServo servoRelicElbow;
    public final HwServo servoRelicClaw;
    public final HwServo servoStick;
    public final HwServo servoDumperClaw;
    public final List<HwServo> servos = new ArrayList<>();

    public final HwSwitch switchSlideDown;
    public final HwSwitch switchSlideUp;
    public final HwSwitch switchRelicSlideIn;
    public final HwSwitch switchDoor;
    public final List<HwSwitch> switchs = new ArrayList<>();

    public final HwLed ledYellow;
    public final HwLed ledGreen;
    public final HwLed ledWhite;
    public final HwLed ledBlue;
    public final HwLed ledRed;
    public final List<HwLed> leds = new ArrayList<>();

    public final Drive drive;

    /**
     * initialize the robot
     *
     * @param op           opMode to run
     * @param isAutonomous if autonomous
     */
    public RobotRevAmped2(OpMode op,
                          boolean isAutonomous) {
        int numberMissing = 0;
        HardwareMap hardwareMap = op.hardwareMap;
        this.telemetry = op.telemetry;

        hardwareMap.logDevices();

        HwMotor driveLeftFront = null;
        try {
            driveLeftFront = new HwMotor(hardwareMap,
                                         "motor_lf",
                                         DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            RobotLog.e("missing: motor_lf " + e.getMessage());
            numberMissing++;
        }
        this.driveLeftFront = driveLeftFront;

        HwMotor driveLeftBack = null;
        try {
            driveLeftBack = new HwMotor(hardwareMap,
                                        "motor_lb",
                                        DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            RobotLog.e("missing: motor_lb " + e.getMessage());
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
            numberMissing++;
        }
        this.driveRightFront = driveRightFront;

        HwMotor driveRightBack = null;
        try {
            driveRightBack = new HwMotor(hardwareMap,
                                         "motor_rb",
                                         DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            RobotLog.e("missing: motor_rb " + e.getMessage());
            numberMissing++;
        }
        this.driveRightBack = driveRightBack;

        HwMotor sweeperLeft = null;
        try {
            sweeperLeft = new HwMotor(hardwareMap,
                                    "motor_sl",
                                    DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            RobotLog.e("missing: motor_sl " + e.getMessage());
            numberMissing++;
        }
        this.sweeperLeft = sweeperLeft;

        HwMotor sweeperRight = null;
        try {
            sweeperRight = new HwMotor(hardwareMap,
                                     "motor_sr",
                                     DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            RobotLog.e("missing: motor_sr " + e.getMessage());
            numberMissing++;
        }
        this.sweeperRight = sweeperRight;

        HwMotor motorSlide = null;
        try {
            motorSlide = new HwMotor(hardwareMap,
                                "motor_slide",
                                DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            RobotLog.e("missing: motor_slide " + e.getMessage());
            numberMissing++;
        }
        this.motorSlide = motorSlide;

        HwMotor motorRelicSlide = null;
        try {
            motorRelicSlide = new HwMotor(hardwareMap,
                                          "motor_relic_slide",
                                          DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            RobotLog.e("missing: motor_relic_slide " + e.getMessage());
            numberMissing++;
        }
        this.motorRelicSlide = motorRelicSlide;

        //Adding new Servos for grabling hand
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

        HwServo servoDoorRight = null;
        try {
            servoDoorRight = new HwServo(hardwareMap,
                                         "servo_dr",
                                         RobotRevAmpedConstants.SERVO_DOOR_IN);
            servos.add(servoDoorRight);
        } catch (Exception e) {
            RobotLog.e("missing: servo_dr " + e.getMessage());
            numberMissing++;
        }
        this.servoDoorRight = servoDoorRight;

        HwServo servoJewel = null;
        try {
            servoJewel = new HwServo(hardwareMap,
                                     "servo_jewel",
                                     RobotRevAmpedConstants.SERVO_JEWEL_IN);
            servos.add(servoJewel);
        } catch (Exception e) {
            RobotLog.e( "missing: servoJewel " + e.getMessage());
            numberMissing++;
        }
        this.servoJewel = servoJewel;

        HwServo servoJewelHit = null;
        try {
            servoJewelHit = new HwServo(hardwareMap,
                                        "servo_jewelhit",
                                        RobotRevAmpedConstants.SERVO_DETECT);
            servos.add(servoJewelHit);
        } catch (Exception e){
            RobotLog.e("missing: servoJewelHit" + e.getMessage());
            numberMissing++;
        }
        this.servoJewelHit = servoJewelHit;

        HwServo servoRelicElbow = null;
        try {
            servoRelicElbow = new HwServo(hardwareMap,
                                          "servo_relic_elbow",
                                          RobotRevAmpedConstants.SERVO_RELIC_ELBOW_REST);
            servos.add(servoRelicElbow);
        } catch (Exception e){
            RobotLog.e("missing: servoRelicElbow" + e.getMessage());
            numberMissing++;
        }
        this.servoRelicElbow = servoRelicElbow;

        HwServo servoRelicClaw = null;
        try {
            servoRelicClaw = new HwServo(hardwareMap,
                                         "servo_relic_claw",
                                         RobotRevAmpedConstants.SERVO_RELIC_CLAW_REST);
            servos.add(servoRelicClaw);
        } catch (Exception e){
            RobotLog.e("missing: servoRelicClaw" + e.getMessage());
            numberMissing++;
        }
        this.servoRelicClaw = servoRelicClaw;

        HwServo servoRangeStick = null;
        try {
            servoRangeStick = new HwServo(hardwareMap,
                                            "servo_stick",
                                            RobotRevAmpedConstants.SERVO_STICK_IN);
            servos.add(servoRangeStick);
        } catch (Exception e) {
            RobotLog.e("missing: servoStick" + e.getMessage());
        }
        this.servoStick = servoRangeStick;


        HwServo servoDumperClaw = null;
        try {
            servoDumperClaw = new HwServo(hardwareMap,
                                          "servo_dumper_claw",
                                          RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
            servos.add(servoDumperClaw);
        } catch (Exception e) {
            RobotLog.e("missing: servoDumperClaw" + e.getMessage());
        }
        this.servoDumperClaw = servoDumperClaw;

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

        HwSwitch switchSlideDown = null;
        try {
            switchSlideDown = new HwSwitch(hardwareMap,
                                           "switch_slidedown",
                                           true);
            switchs.add(switchSlideDown);
        } catch (Exception e) {
            RobotLog.e("missing: digital channel " + e.getMessage());
            numberMissing++;
        }
        this.switchSlideDown = switchSlideDown;

        HwSwitch switchRelicSlideIn = null;
        try {
            switchRelicSlideIn = new HwSwitch(hardwareMap,
                                              "switch_relic_slidein",
                                              true);
            switchs.add(switchRelicSlideIn);
        } catch (Exception e) {
            RobotLog.e("missing: digital channel " + e.getMessage());
            numberMissing++;
        }
        this.switchRelicSlideIn = switchRelicSlideIn;

        HwSwitch switchDoor = null;
        try {
            switchDoor = new HwSwitch(hardwareMap,
                    "door_switch",
                    true);
            switchs.add(switchDoor);
        } catch (Exception e) {
            RobotLog.e("missing: digital channel " + e.getMessage());
            numberMissing++;
        }
        this.switchDoor = switchDoor;
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
        ///Initialize the Sonar sensor used in this robot
        sonarRF = new HwSonarAnalog(hardwareMap,
                                    "sonar_rf",
                                    HwSonarAnalog.SCALE_MAX_XL);
        sonarAnalogs.add(sonarRF);
        sonarL = new HwSonarAnalog(hardwareMap,
                                   "sonar_l",
                                   HwSonarAnalog.SCALE_MAX_XL);
        sonarAnalogs.add(sonarL);
        sonarR = new HwSonarAnalog(hardwareMap,
                "sonar_r",
                HwSonarAnalog.SCALE_MAX_LV);
        sonarAnalogs.add(sonarR);

        /*sonarTest = new HwSonarAnalog(hardwareMap,
                "sonar_test",
                HwSonarAnalog.SCALE_MAX_XL);*/

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
        return new MecanumDrive(driveLeftFront,
                                driveLeftBack,
                                driveRightFront,
                                driveRightBack);
    }

    /**
     * initialize tank drive
     *
     * @return TankDrive
     */
    public MecanumDrive getMecanumDrive() {
        return (MecanumDrive) this.drive;
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
