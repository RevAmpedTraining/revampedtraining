package com.revAmped.components;

import com.revAmped.config.RobotConstants;
import com.revAmped.config.RobotConstants.COLOR_SENSOR;
import com.revAmped.config.RobotConstants.LED;
import com.revAmped.config.RobotConstants.SONAR;
import com.revAmped.sensors.MultiplexColorSensor;
import com.revAmped.util.HwLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * Robot definition for teleop
 */
public class Robot
{
    protected Telemetry telemetry;

    public final DeviceInterfaceModule dim = null;

    public final MultiplexColorSensor colorSensor;
    public final HwGyro gyroSensor;

    public final HwMotor driveLeftFront;
    public final HwMotor driveLeftBack;
    public final HwMotor driveRightFront;
    public final HwMotor driveRightBack;
    public final HwMotor spinnerLeft;
    public final HwMotor spinnerRight;
    public final HwMotor roller;
    public final HwMotor slide;

    public final HwServo servoLeftFront;
    public final HwServo servoLeftBack;
    public final HwServo servoRightFront;
    public final HwServo servoRightBack;
    public final HwServo servoTrigger;
    public final HwServo servoBeacon;
    public final HwServo servoCapBall;
    public final List<HwServo> servos = new ArrayList<>();

    public final HwServo servoSlide = null;

    public final HwSwitch switchSlideUp;
    public final HwSwitch switchSlideDown;
    public final List<HwSwitch> switchs = new ArrayList<>();

    public final HwLed ledYellow;
    public final HwLed ledGreen;
    public final HwLed ledWhite;
    public final HwLed ledBlue;
    public final HwLed ledRed;
    public final List<HwLed> leds = new ArrayList<>();

    public final HwSonarAnalog sonarFront;
    public final HwSonarAnalog sonarLeft;
    public final HwSonarAnalog sonarRight;
    public final List<HwSonarAnalog> sonars = new ArrayList<>();

    public final Drive drive;

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
                                         "driveLF",
                                         DcMotor.Direction.REVERSE,
                                         isAutonomous ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            HwLog.error("missing: driveLF " + e.getMessage());
            numberMissing++;
        }
        this.driveLeftFront = driveLeftFront;

        HwMotor driveLeftBack = null;
        try {
            driveLeftBack = new HwMotor(hardwareMap,
                                        "driveLB",
                                        DcMotor.Direction.REVERSE,
                                        isAutonomous ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            HwLog.error( "missing: driveLB " + e.getMessage());
            numberMissing++;
        }
        this.driveLeftBack = driveLeftBack;

        HwMotor driveRightFront = null;
        try {
            driveRightFront = new HwMotor(hardwareMap,
                                          "driveRF",
                                          DcMotor.Direction.FORWARD,
                                          isAutonomous ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            HwLog.error( "missing: driveRF " + e.getMessage());
            numberMissing++;
        }
        this.driveRightFront = driveRightFront;

        HwMotor driveRightBack = null;
        try {
            driveRightBack = new HwMotor(hardwareMap,
                                         "driveRB",
                                         DcMotor.Direction.FORWARD,
                                         isAutonomous ? DcMotor.RunMode.RUN_USING_ENCODER : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            HwLog.error( "missing: driveRB " + e.getMessage());
            numberMissing++;
        }
        this.driveRightBack = driveRightBack;

        HwMotor spinnerLeft = null;
        try {
            spinnerLeft = new HwMotor(hardwareMap,
                                    "gunL",
                                    DcMotor.Direction.REVERSE,
                                    DcMotor.RunMode.RUN_USING_ENCODER);
            spinnerLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e)
        {
            HwLog.error( "missing: gunL " + e.getMessage());
            numberMissing++;
        }
        this.spinnerLeft = spinnerLeft;

        HwMotor spinnerRight = null;
        try {
            spinnerRight = new HwMotor(hardwareMap,
                                     "gunR",
                                     DcMotor.Direction.FORWARD,
                                     DcMotor.RunMode.RUN_USING_ENCODER);
            spinnerRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e)
        {
            HwLog.error( "missing: gunR " + e.getMessage());
            numberMissing++;
        }
        this.spinnerRight = spinnerRight;

        HwMotor roller = null;
        try {
            roller = new HwMotor(hardwareMap,
                                 "roller",
                                 DcMotor.Direction.FORWARD,
                                 DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            roller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e)
        {
            HwLog.error( "missing: roller " + e.getMessage());
            numberMissing++;
        }
        this.roller = roller;

        HwMotor slide = null;
        try {
            slide = new HwMotor(hardwareMap,
                                 "slide",
                                 DcMotor.Direction.FORWARD,
                                 DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e)
        {
            HwLog.error( "missing: slide " + e.getMessage());
            numberMissing++;
        }
        this.slide = slide;

        HwServo servoRightFront = null;
        try {
            servoRightFront = new HwServo(hardwareMap,
                    "servoRF",
                    RobotConstants.SERVO_RIGHTFRONT_START);
            servos.add(servoRightFront);
        } catch (Exception e) {
            HwLog.error( "missing: servoRF " + e.getMessage());
            numberMissing++;
        }
        this.servoRightFront = servoRightFront;

        HwServo servoRightBack = null;
        try {
            servoRightBack = new HwServo(hardwareMap,
                    "servoRB",
                    RobotConstants.SERVO_RIGHTBACK_START);
            servos.add(servoRightBack);
        } catch (Exception e) {
            HwLog.error( "missing: servoRB " + e.getMessage());
            numberMissing++;
        }
        this.servoRightBack = servoRightBack;

        HwServo servoLeftFront = null;
        try {
            servoLeftFront = new HwServo(hardwareMap,
                                          "servoLF",
                                          RobotConstants.SERVO_LEFTFRONT_START);
            servos.add(servoLeftFront);
        } catch (Exception e) {
            HwLog.error( "missing: servoLF " + e.getMessage());
            numberMissing++;
        }
        this.servoLeftFront = servoLeftFront;

        HwServo servoLeftBack = null;
        try {
            servoLeftBack = new HwServo(hardwareMap,
                    "servoLB",
                    RobotConstants.SERVO_LEFTBACK_START);
            servos.add(servoLeftBack);
        } catch (Exception e) {
            HwLog.error( "missing: servoLB " + e.getMessage());
            numberMissing++;
        }
        this.servoLeftBack = servoLeftBack;

        HwServo servoTrigger = null;
        try {
            servoTrigger = new HwServo(hardwareMap,
                    "servoTG",
                    RobotConstants.SERVO_TRIGGER_IN);
            servos.add(servoTrigger);
        } catch (Exception e) {
            HwLog.error( "missing: servoTG " + e.getMessage());
            numberMissing++;
        }
        this.servoTrigger = servoTrigger;

        HwServo servoBeacon = null;
        try {
            servoBeacon = new HwServo(hardwareMap,
                                       "servoBC",
                                       RobotConstants.SERVO_BEACON_STOP);
            servos.add(servoBeacon);
        } catch (Exception e) {
            HwLog.error( "missing: servoBC " + e.getMessage());
            numberMissing++;
        }
        this.servoBeacon = servoBeacon;

        HwServo servoCapBall = null;
        try {
            servoCapBall = new HwServo(hardwareMap,
                                      "servoCB",
                                      RobotConstants.SERVO_CAP_BALL_OPEN);
            servos.add(servoCapBall);
        } catch (Exception e) {
            HwLog.error( "missing: servoCB " + e.getMessage());
            numberMissing++;
        }
        this.servoCapBall = servoCapBall;

        DeviceInterfaceModule dim = null;
        try {
            dim = hardwareMap.deviceInterfaceModule.get("Device Interface Module");
        } catch (Exception e) {
            HwLog.error( "missing: device interface module " + e.getMessage());
            numberMissing++;
        }

        switchSlideUp = new HwSwitch(dim,
                                    "switch_slide_up",
                                    RobotConstants.SWITCH_SLIDE_UP_CHANNEL,
                                    true);
        switchs.add(switchSlideUp);
        switchSlideDown = new HwSwitch(dim,
                                     "switch_slide_down",
                                     RobotConstants.SWITCH_SLIDE_DOWN_CHANNEL,
                                     true);
        switchs.add(switchSlideDown);
        ledYellow = new HwLed(dim,
                              LED.YELLOW);
        leds.add(ledYellow);
        ledGreen = new HwLed(dim,
                             LED.GREEN);
        leds.add(ledGreen);
        ledWhite = new HwLed(dim,
                             LED.WHITE);
        leds.add(ledWhite);
        ledBlue = new HwLed(dim,
                            LED.BLUE);
        leds.add(ledBlue);
        ledRed = new HwLed(dim,
                           LED.RED);
        leds.add(ledRed);

        sonarFront = new HwSonarAnalog(dim,
                                      SONAR.FRONT,
                                      HwSonarAnalog.SCALE_MAX_XL);
        sonars.add(sonarFront);
        sonarLeft = new HwSonarAnalog(dim,
                                      SONAR.LEFT,
                                      HwSonarAnalog.SCALE_MAX_LV);
        sonars.add(sonarLeft);
        sonarRight = new HwSonarAnalog(dim,
                                       SONAR.RIGHT,
                                       HwSonarAnalog.SCALE_MAX_LV);
        sonars.add(sonarRight);

        MultiplexColorSensor colorSensor = null;
        if(isAutonomous) {
            try {
                colorSensor = new MultiplexColorSensor(op.hardwareMap,
                                                       "mux",
                                                       "color_sensor",
                                                       MultiplexColorSensor.MUX_ADDRESS_0,
                                                       COLOR_SENSOR.values());
            } catch (Exception e) {
                HwLog.error("missing: mux_color_sensor " + e.getMessage());
                numberMissing++;
            }
        }
        this.colorSensor = colorSensor;

        HwGyro gyroSensor = null;
        if(isAutonomous) {
            try {
                gyroSensor = new HwNavGyro(dim,
                                           RobotConstants.NAVX_DIM_I2C_PORT);
                while (gyroSensor.isCalibrating()) {
                    telemetry.addData("Gyro", "Calibrating");
                    telemetry.update();
                    Thread.sleep(50);
                }
                // is connected
                while (!gyroSensor.isConnected()) {
                    telemetry.addData("Gyro", "please check connection.");
                    telemetry.update();
                    Thread.sleep(100);
                }
            } catch (Exception e) {
                HwLog.error("missing: gyro_sensor " + e.getMessage());
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
    public void close ()
    {
        if (this.gyroSensor != null) {
            this.gyroSensor.close();
        }
    }

    /**
     * subclass override this method
     * @return Drive
     */
    protected Drive createDrive () {
        return new SwerveDrive(driveLeftFront,
                driveLeftBack,
                driveRightFront,
                driveRightBack,
                servoLeftFront,
                servoLeftBack,
                servoRightFront,
                servoRightBack);
    }

    /**
     * initialize swerve drive
     * @return SwerveDrive
     */
    public SwerveDrive getSwerveDrive () {
        return (SwerveDrive)this.drive;
    }

    /**
     * update LEDs
     */
    public void drawLed () {
        for (HwLed led: leds) {
            led.draw();
        }
    }

}
