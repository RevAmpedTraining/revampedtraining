package com.revAmped.components;

import com.revAmped.config.RobotConstants;
import com.revAmped.util.HwLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * Robot definition for teleop
 */
public class Robot
{
    protected Telemetry telemetry;

    public final HwColorSensor colorLeftOut;
    public final HwColorSensor colorLeftIn;
    public final HwColorSensor colorRightOut;
    public final HwColorSensor colorRightIn;
    public final HwGyro gyroSensor;

    public final HwMotor driveLeftFront;
    public final HwMotor driveLeftBack;
    public final HwMotor driveRightFront;
    public final HwMotor driveRightBack;
    public final HwMotor slideLeft;
    public final HwMotor slideRight;
    public final HwMotor roller;

    public final HwServo servoBucketTurn;
    public final HwServo servoFrontDoor;
    public final HwServo servoBackDoor;
    public final HwServo servoBucket;
    public final HwServo servoHookLeft;
    public final HwServo servoHookRight;
    public final HwServo servoSlide;
    public final HwServo servoClimber;
    public final HwServo servoWingLeft;
    public final HwServo servoWingRight;
    public final HwServo servoBeacon;
    public final List<HwServo> servos = new ArrayList<>();

    public final HwSwitch switchBucket;
    public final HwSwitch switchSlideDown;
    public final HwSwitch switchSlideUp;

    public final HwLed ledYellow;
    public final HwLed ledGreen;
    public final HwLed ledWhite;
    public final HwLed ledBlue;
    public final HwLed ledRed;
    public final List<HwLed> leds = new ArrayList<>();

    public final Drive drive;

    /** AndyMark NeveRest 40 motor specs */
    public final static int AM40_ENCODER_TICK_PER_REVOLUTION = 1120;
    public final static int TETRIX_ENCODER_TICK_PER_REVOLUTION = 1440;
    public final static int AM40_MOTOR_RPM = 129;

    public final static float AM40_ENCODER_TICK_PER_CM =
        (float)(RobotConstants.WHEEL_TO_MOTOR_GEAR_RATIO * AM40_ENCODER_TICK_PER_REVOLUTION /
            (Math.PI * RobotConstants.WHEEL_DIAMETER_CM));
    public final static float AM40_ENCODER_TICK_PER_INCH = AM40_ENCODER_TICK_PER_CM * 2.54f; //104.645
    public final static float AM40_MILLISECOND_PER_TICK =
        (float) (60000.0f / ((float) AM40_ENCODER_TICK_PER_REVOLUTION * AM40_MOTOR_RPM));

    public final static float AM40_ENCODER_RATIO =
        ((float) AM40_ENCODER_TICK_PER_REVOLUTION) / ((float) TETRIX_ENCODER_TICK_PER_REVOLUTION);

    /**
     * initialize the robot
     * @param op opMode to run
     * @param isAutonomous if autonomous
     */
    public Robot (OpMode op,
                  boolean isAutonomous)
    {
        int numberMissing = 0;
        HardwareMap hardwareMap = op.hardwareMap;
        this.telemetry = op.telemetry;

        hardwareMap.logDevices();

        HwMotor driveLeftFront = null;
        try {
            driveLeftFront = new HwMotor(hardwareMap,
                                         "motor_lf",
                                         DcMotor.Direction.FORWARD,
                                         DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            HwLog.error("missing: motor_lf " + e.getMessage());
            numberMissing++;
        }
        this.driveLeftFront = driveLeftFront;

        HwMotor driveLeftBack = null;
        try {
            driveLeftBack = new HwMotor(hardwareMap,
                                        "motor_lb",
                                        DcMotor.Direction.FORWARD,
                                        DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            HwLog.error( "missing: motor_lb " + e.getMessage());
            numberMissing++;
        }
        this.driveLeftBack = driveLeftBack;

        HwMotor driveRightFront = null;
        try {
            driveRightFront = new HwMotor(hardwareMap,
                                          "motor_rf",
                                          DcMotor.Direction.REVERSE,
                                          DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            HwLog.error( "missing: motor_rf " + e.getMessage());
            numberMissing++;
        }
        this.driveRightFront = driveRightFront;

        HwMotor driveRightBack = null;
        try {
            driveRightBack = new HwMotor(hardwareMap,
                                         "motor_rb",
                                         DcMotor.Direction.REVERSE,
                                         DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            HwLog.error( "missing: motor_rb " + e.getMessage());
            numberMissing++;
        }
        this.driveRightBack = driveRightBack;

        HwMotor slideLeft = null;
        try {
            slideLeft = new HwMotor(hardwareMap,
                                    "motor_sl",
                                    DcMotor.Direction.FORWARD,
                                    DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e)
        {
            HwLog.error( "missing: motor_sl " + e.getMessage());
            numberMissing++;
        }
        this.slideLeft = slideLeft;

        HwMotor slideRight = null;
        try {
            slideRight = new HwMotor(hardwareMap,
                                     "motor_sr",
                                     DcMotor.Direction.REVERSE,
                                     DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e)
        {
            HwLog.error( "missing: motor_sr " + e.getMessage());
            numberMissing++;
        }
        this.slideRight = slideRight;

        HwMotor roller = null;
        try {
            roller = new HwMotor(hardwareMap,
                                 "motor_ro",
                                 DcMotor.Direction.FORWARD,
                                 DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e)
        {
            HwLog.error( "missing: motor_ro " + e.getMessage());
            numberMissing++;
        }
        this.roller = roller;

        HwServo servoFrontDoor = null;
        try {
            servoFrontDoor = new HwServo(hardwareMap,
                                    "servo_fdoor",
                                    RobotConstants.SERVO_FDOOR_OPEN);
            servos.add(servoFrontDoor);
        } catch (Exception e) {
            HwLog.error( "missing: servo_door " + e.getMessage());
            numberMissing++;
        }
        this.servoFrontDoor = servoFrontDoor;

        HwServo servoBackDoor = null;
        try {
            servoBackDoor = new HwServo(hardwareMap,
                    "servo_bdoor",
                    RobotConstants.SERVO_BDOOR_OPEN);
            servos.add(servoBackDoor);
        } catch (Exception e) {
            HwLog.error( "missing: servo_bdoor " + e.getMessage());
            numberMissing++;
        }
        this.servoBackDoor = servoBackDoor;

        HwServo servoBucket = null;
        try {
            servoBucket = new HwServo(hardwareMap,
                    "servo_bucket",
                    RobotConstants.SERVO_BUCKET_CLOSED);
            servos.add(servoBucket);
        } catch (Exception e) {
            HwLog.error( "missing: servo_bucket " + e.getMessage());
            numberMissing++;
        }
        this.servoBucket = servoBucket;

        HwServo servoBucketTurn = null;
        try {
            servoBucketTurn = new HwServo(hardwareMap,
                                          "servo_bturn",
                                          RobotConstants.SERVO_BUCKET_TURN_CENTER);
            servos.add(servoBucketTurn);
        } catch (Exception e) {
            HwLog.error( "missing: servo_bturn " + e.getMessage());
            numberMissing++;
        }
        this.servoBucketTurn = servoBucketTurn;

        HwServo servoHookLeft = null;
        try {
            servoHookLeft = new HwServo(hardwareMap,
                                        "servo_hookl",
                                        RobotConstants.SERVO_HOOKL_UP);
            servos.add(servoHookLeft);
        } catch (Exception e) {
            HwLog.error( "missing: servo_hookl " + e.getMessage());
            numberMissing++;
        }
        this.servoHookLeft = servoHookLeft;

        HwServo servoHookRight = null;
        try {
            servoHookRight = new HwServo(hardwareMap,
                                         "servo_hookr",
                                         RobotConstants.SERVO_HOOKR_UP);
            servos.add(servoHookRight);
        } catch (Exception e) {
            HwLog.error( "missing: servo_hookr " + e.getMessage());
            numberMissing++;
        }
        this.servoHookRight = servoHookRight;

        HwServo servoSlide = null;
        try {
            servoSlide = new HwServo(hardwareMap,
                                     "servo_slide",
                                     RobotConstants.SERVO_SLIDE_START);
            servos.add(servoSlide);
        } catch (Exception e) {
            HwLog.error( "missing: servo_slide " + e.getMessage());
            numberMissing++;
        }
        this.servoSlide = servoSlide;

        HwServo servoClimber = null;
        try {
            servoClimber = new HwServo(hardwareMap,
                                       "servo_climber",
                                       RobotConstants.SERVO_CLIMBER_IN);
            servos.add(servoClimber);
        } catch (Exception e) {
            HwLog.error( "missing: servo_climber " + e.getMessage());
            numberMissing++;
        }
        this.servoClimber = servoClimber;

        HwServo servoWingLeft = null;
        try {
            servoWingLeft = new HwServo(hardwareMap,
                                        "servo_wingl",
                                        RobotConstants.SERVO_WINGL_IN);
            servos.add(servoWingLeft);
        } catch (Exception e)
        {
            HwLog.error( "missing: servo_wingl " + e.getMessage());
            numberMissing++;
        }
        this.servoWingLeft = servoWingLeft;

        HwServo servoWingRight = null;
        try {
            servoWingRight = new HwServo(hardwareMap,
                                         "servo_wingr",
                                         RobotConstants.SERVO_WINGR_IN);
            servos.add(servoWingRight);
        } catch (Exception e)
        {
            HwLog.error( "missing: servo_wingr " + e.getMessage());
            numberMissing++;
        }
        this.servoWingRight = servoWingRight;

        HwServo servoBeacon = null;
        try {
            servoBeacon = new HwServo(hardwareMap,
                                      "servo_beacon",
                                      RobotConstants.SERVO_BEACON_MID);
            servos.add(servoBeacon);
        } catch (Exception e)
        {
            HwLog.error( "missing: servo_beacon " + e.getMessage());
            numberMissing++;
        }
        this.servoBeacon = servoBeacon;

        DeviceInterfaceModule dim = null;
        try {
            dim = hardwareMap.deviceInterfaceModule.get("Device Interface Module");
            dim.setDigitalChannelMode(RobotConstants.LED_YELLOW_CHANNEL, DigitalChannelController.Mode.OUTPUT);
            dim.setDigitalChannelMode(RobotConstants.LED_WHITE_CHANNEL, DigitalChannelController.Mode.OUTPUT);
            dim.setDigitalChannelMode(RobotConstants.LED_GREEN_CHANNEL, DigitalChannelController.Mode.OUTPUT);
            dim.setDigitalChannelMode(RobotConstants.LED_BLUE_CHANNEL, DigitalChannelController.Mode.OUTPUT);
            dim.setDigitalChannelMode(RobotConstants.LED_RED_CHANNEL, DigitalChannelController.Mode.OUTPUT);
        } catch (Exception e) {
            HwLog.error( "missing: device interface module " + e.getMessage());
            numberMissing++;
        }

        switchBucket = new HwSwitch(dim,
                                    "switch_bucket",
                                    RobotConstants.SWITCH_BUCKET_CHANNEL,
                                    false);
        switchSlideDown = new HwSwitch(dim,
                                       "switch_slide_down",
                                       RobotConstants.SWITCH_SLIDE_DOWN_CHANNEL,
                                       false);
        switchSlideUp = new HwSwitch(dim,
                                     "switch_slide_up",
                                     RobotConstants.SWITCH_SLIDE_UP_CHANNEL,
                                     false);

        ledYellow = new HwLed(dim,
                              "led_yellow",
                              RobotConstants.LED_YELLOW_CHANNEL);
        leds.add(ledYellow);
        ledGreen = new HwLed(dim,
                             "led_green",
                             RobotConstants.LED_GREEN_CHANNEL);
        leds.add(ledGreen);
        ledWhite = new HwLed(dim,
                             "led_white",
                             RobotConstants.LED_WHITE_CHANNEL);
        leds.add(ledWhite);
        ledBlue = new HwLed(dim,
                            "led_blue",
                            RobotConstants.LED_BLUE_CHANNEL);
        leds.add(ledBlue);
        ledRed = new HwLed(dim,
                           "led_red",
                           RobotConstants.LED_RED_CHANNEL);
        leds.add(ledRed);

        HwColorSensor colorLeftOut = null;
        try {
            colorLeftOut = new HwColorSensor(hardwareMap,
                                             "color_lo");
            colorLeftOut.setI2cAddress(new I2cAddr(RobotConstants.COLOR_LEFT_OUT_ADDRESS));
            colorLeftOut.enableLed(false);
        } catch (Exception e) {
            HwLog.error( "missing: color_lo " + e.getMessage());
            numberMissing++;
        }
        this.colorLeftOut = colorLeftOut;

        HwColorSensor colorLeftIn = null;
        try {
            colorLeftIn = new HwColorSensor(hardwareMap,
                                            "color_li");
            colorLeftIn.setI2cAddress(new I2cAddr(RobotConstants.COLOR_LEFT_IN_ADDRESS));
            colorLeftIn.enableLed(false);
        } catch (Exception e) {
            HwLog.error( "missing: color_li " + e.getMessage());
            numberMissing++;
        }
        this.colorLeftIn = colorLeftIn;

        HwColorSensor colorRightIn = null;
        try {
            colorRightIn = new HwColorSensor(hardwareMap,
                                             "color_ri");
            colorRightIn.setI2cAddress(new I2cAddr(RobotConstants.COLOR_RIGHT_IN_ADDRESS));
            colorRightIn.enableLed(false);
        } catch (Exception e) {
            HwLog.error( "missing: color_ri " + e.getMessage());
            numberMissing++;
        }
        this.colorRightIn = colorRightIn;

        HwColorSensor colorRightOut = null;
        try {
            colorRightOut = new HwColorSensor(hardwareMap,
                                              "color_ro");
            colorRightOut.setI2cAddress(new I2cAddr(RobotConstants.COLOR_RIGHT_OUT_ADDRESS));
            colorRightOut.enableLed(false);
        } catch (Exception e) {
            HwLog.error( "missing: color_ro " + e.getMessage());
            numberMissing++;
        }
        this.colorRightOut = colorRightOut;

        HwGyro gyroSensor = null;
        if(isAutonomous) {
            try {
                gyroSensor = new HwMRGyro(hardwareMap,
                                          "gyro_sensor");
                while (gyroSensor.isCalibrating()) {
                    telemetry.addData("Gyro", "Calibrating");
                    Thread.sleep(50);
                }
            } catch (Exception e) {
                HwLog.error("missing: gyro_sensor " + e.getMessage());
                numberMissing++;
            }
        }
        this.gyroSensor = gyroSensor;

        this.drive = createDrive();

        op.gamepad1.reset();
        op.gamepad1.setJoystickDeadzone(0.15f);
        op.gamepad2.reset();
        op.gamepad2.setJoystickDeadzone(0.15f);

        telemetry.addData("Missing Devices", numberMissing);
    }

    /**
     * Robot and sensor shut down
     */
    public void close ()
    {

    }

    /**
     * subclass override this method
     * @return Drive
     */
    protected Drive createDrive () {
        return new TankDrive(driveLeftFront,
                             driveLeftBack,
                             driveRightFront,
                             driveRightBack);
    }

    /**
     * initialize tank drive
     * @return TankDrive
     */
    public TankDrive getTankDrive () {
        return (TankDrive)this.drive;
    }

    /**
     * update LEDs
     */
    public void drawLed () {
        for (HwLed led: leds) {
            led.draw();
        }
    }

    /**
     *
     * @return
     */
    public Boolean isLeftRed() {
        Boolean bLeftRed = null;
        if (colorLeftOut.isBlue()) {
            bLeftRed = Boolean.FALSE;
        }
        else if (colorRightOut.isBlue()) {
            bLeftRed = Boolean.TRUE;
        }
        else if (colorLeftOut.isRed()) {
            bLeftRed = Boolean.TRUE;
        }
        else if (colorRightOut.isRed()) {
            bLeftRed = Boolean.FALSE;
        }
        else if (colorLeftIn.isBlue()) {
            bLeftRed = Boolean.FALSE;
        }
        else if (colorRightIn.isBlue()) {
            bLeftRed = Boolean.TRUE;
        }
        else if (colorLeftIn.isRed()) {
            bLeftRed = Boolean.TRUE;
        }
        else if (colorRightIn.isRed()) {
            bLeftRed = Boolean.FALSE;
        }
        return bLeftRed;
    }
}
