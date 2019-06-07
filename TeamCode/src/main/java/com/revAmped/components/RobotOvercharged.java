package com.revAmped.components;

import com.revAmped.config.RobotOverchargedConstants;
import com.revAmped.config.SwerveDriveConstants;
import com.revAmped.sensors.VuMarkSensing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

/**
 * RobotOvercharged definition for teleop
 */
public class RobotOvercharged
{
    protected Telemetry telemetry;
    public final HwDistanceSensor jewelDistanceSensor;
    public final VuMarkSensing relicRecoveryVuMark;
    public final List<HwDistanceSensor> distanceSensors = new ArrayList<>();
    //public final HwColorSensor jewelColorSensor;
    //public final List<HwColorSensor> colorSensors = new ArrayList<>();

    public final HwGyro gyroSensor;
    public final HwMotor driveLeftFront;
    public final HwMotor driveLeftBack;
    public final HwMotor driveRightFront;
    public final HwMotor driveRightBack;
    /*
    public final HwMotor spinnerLeft;
    public final HwMotor spinnerRight;
    public final HwMotor roller;
    */
    public final HwMotor slideLeft;
    public final HwMotor slideRight;

    public final HwServo servoLeftFront;
    public final HwServo servoLeftBack;
    public final HwServo servoRightFront;
    public final HwServo servoRightBack;
    public final HwServo servoClawLeft;
    public final HwServo servoClawRight;
    public final HwServo servoJewel;
    /*
    public final HwServo servoCapBall;
    */
    public final List<HwServo> servos = new ArrayList<>();

    public final HwSwitch switchSlideUp;
    public final HwSwitch switchSlideDown;
    public final List<HwSwitch> switchs = new ArrayList<>();

    public final HwLed ledYellow;
    public final HwLed ledGreen;
    public final HwLed ledWhite;
    public final HwLed ledBlue;
    public final HwLed ledRed;
    public final List<HwLed> leds = new ArrayList<>();

    /*
    public final HwSonarAnalog sonarFront;
    public final HwSonarAnalog sonarLeft;
    public final HwSonarAnalog sonarRight;
    public final List<HwSonarAnalog> sonars = new ArrayList<>();
    */

    public final Drive drive;

    /**
     * initialize the robot
     * @param op opMode to run
     * @param isAutonomous if autonomous
     */
    public RobotOvercharged(OpMode op,
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
                                         DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            RobotLog.e("missing: driveLF " + e.getMessage());
            numberMissing++;
        }
        this.driveLeftFront = driveLeftFront;

        HwMotor driveLeftBack = null;
        try {
            driveLeftBack = new HwMotor(hardwareMap,
                                        "driveLB",
                                        DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            RobotLog.e("missing: driveLB " + e.getMessage());
            numberMissing++;
        }
        this.driveLeftBack = driveLeftBack;

        HwMotor driveRightFront = null;
        try {
            driveRightFront = new HwMotor(hardwareMap,
                                          "driveRF",
                                          DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            RobotLog.e( "missing: driveRF " + e.getMessage());
            numberMissing++;
        }
        this.driveRightFront = driveRightFront;

        HwMotor driveRightBack = null;
        try {
            driveRightBack = new HwMotor(hardwareMap,
                                         "driveRB",
                                         DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            RobotLog.e( "missing: driveRB " + e.getMessage());
            numberMissing++;
        }
        this.driveRightBack = driveRightBack;
        /*
        HwMotor spinnerLeft = null;
        try {
            spinnerLeft = new HwMotor(hardwareMap,
                                    "gunL",
                                    DcMotor.Direction.REVERSE);
            spinnerLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e)
        {
            RobotLog.e( "missing: gunL " + e.getMessage());
            numberMissing++;
        }
        this.spinnerLeft = spinnerLeft;

        HwMotor spinnerRight = null;
        try {
            spinnerRight = new HwMotor(hardwareMap,
                                     "gunR",
                                     DcMotor.Direction.FORWARD);
            spinnerRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e)
        {
            RobotLog.e( "missing: gunR " + e.getMessage());
            numberMissing++;
        }
        this.spinnerRight = spinnerRight;

        HwMotor roller = null;
        try {
            roller = new HwMotor(hardwareMap,
                                 "roller",
                                 DcMotor.Direction.FORWARD);
            roller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e)
        {
            RobotLog.e( "missing: roller " + e.getMessage());
            numberMissing++;
        }
        this.roller = roller;
        */
        HwMotor slideLeft = null;
        try {
            slideLeft = new HwMotor(hardwareMap,
                                 "slideL",
                                 DcMotor.Direction.REVERSE);
        } catch (Exception e)
        {
            RobotLog.e( "missing: slideL " + e.getMessage());
            numberMissing++;
        }
        this.slideLeft = slideLeft;

        HwMotor slideRight = null;
        try {
            slideRight = new HwMotor(hardwareMap,
                    "slideR",
                    DcMotor.Direction.FORWARD);
        } catch (Exception e)
        {
            RobotLog.e( "missing: slideR " + e.getMessage());
            numberMissing++;
        }
        this.slideRight = slideRight;

        HwServo servoRightFront = null;
        try {
            servoRightFront = new HwServo(hardwareMap,
                                          "servoRF",
                                          SwerveDriveConstants.SERVO_RIGHTFRONT_START);
            servos.add(servoRightFront);
        } catch (Exception e) {
            RobotLog.e( "missing: servoRF " + e.getMessage());
            numberMissing++;
        }
        this.servoRightFront = servoRightFront;

        HwServo servoRightBack = null;
        try {
            servoRightBack = new HwServo(hardwareMap,
                                         "servoRB",
                                         SwerveDriveConstants.SERVO_RIGHTBACK_START);
            servos.add(servoRightBack);
        } catch (Exception e) {
            RobotLog.e( "missing: servoRB " + e.getMessage());
            numberMissing++;
        }
        this.servoRightBack = servoRightBack;

        HwServo servoLeftFront = null;
        try {
            servoLeftFront = new HwServo(hardwareMap,
                                         "servoLF",
                                         SwerveDriveConstants.SERVO_LEFTFRONT_START);
            servos.add(servoLeftFront);
        } catch (Exception e) {
            RobotLog.e( "missing: servoLF " + e.getMessage());
            numberMissing++;
        }
        this.servoLeftFront = servoLeftFront;

        HwServo servoLeftBack = null;
        try {
            servoLeftBack = new HwServo(hardwareMap,
                                        "servoLB",
                                        SwerveDriveConstants.SERVO_LEFTBACK_START);
            servos.add(servoLeftBack);
        } catch (Exception e) {
            RobotLog.e( "missing: servoLB " + e.getMessage());
            numberMissing++;
        }
        this.servoLeftBack = servoLeftBack;

        HwServo servoClawL = null;
        try {
            servoClawL = new HwServo(hardwareMap,
                    "servoClawLeft",
                    RobotOverchargedConstants.SERVO_CLAW_LEFT_OUT);
            servos.add(servoClawL);
        } catch (Exception e) {
            RobotLog.e( "missing: servoClawLeft " + e.getMessage());
            numberMissing++;
        }
        this.servoClawLeft = servoClawL;

        HwServo servoClawR = null;
        try {
            servoClawR = new HwServo(hardwareMap,
                                     "servoClawRight",
                                     RobotOverchargedConstants.SERVO_CLAW_RIGHT_OUT);
            servos.add(servoClawR);
        } catch (Exception e) {
            RobotLog.e( "missing: servoClawRight " + e.getMessage());
            numberMissing++;
        }
        this.servoClawRight = servoClawR;

        HwServo servoJewel = null;
        try {
            servoJewel = new HwServo(hardwareMap,
                                     "servoJewel",
                                     RobotOverchargedConstants.SERVO_JEWEL_IN);
            servos.add(servoJewel);
        } catch (Exception e) {
            RobotLog.e( "missing: servoJewel " + e.getMessage());
            numberMissing++;
        }
        this.servoJewel = servoJewel;

        switchSlideUp = new HwSwitch(hardwareMap,
                                     "switch_slide_up",
                                     true);
        switchs.add(switchSlideUp);

        switchSlideDown = new HwSwitch(hardwareMap,
                                       "switch_slide_down",
                                       true);
        switchs.add(switchSlideDown);

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
        sonarFront = new HwSonarAnalog(hardwareMap,
                                      "sonar_front",
                                      HwSonarAnalog.SCALE_MAX_XL);
        sonars.add(sonarFront);
        sonarLeft = new HwSonarAnalog(hardwareMap,
                                      "sonar_left",
                                      HwSonarAnalog.SCALE_MAX_LV);
        sonars.add(sonarLeft);
        sonarRight = new HwSonarAnalog(hardwareMap,
                                       "sonar_right",
                                       HwSonarAnalog.SCALE_MAX_LV);
        sonars.add(sonarRight);
        */

        /*
        HwColorSensor jewelColorSensor = null;
        if(isAutonomous) {
            try {
                jewelColorSensor = new HwColorSensor(hardwareMap, "jewel_color");
            } catch (Exception e) {
                RobotLog.e("missing: jewel_color_sensor " + e.getMessage());
                numberMissing++;
            }
        }
        this.jewelColorSensor = jewelColorSensor;
        colorSensors.add(jewelColorSensor);
        */

        HwDistanceSensor jewelDistanceSensor = null;
        if(isAutonomous) {
            try {
                jewelDistanceSensor = new HwDistanceSensor(hardwareMap, "jewel_color_distance");
            } catch (Exception e) {
                RobotLog.e("mising: jewel_distance_sensor" + e.getMessage());
                numberMissing++;
            }
        }
        this.jewelDistanceSensor = jewelDistanceSensor;
        distanceSensors.add(jewelDistanceSensor);

        VuMarkSensing vuMarkSensing = null;
        if (isAutonomous) {
            try {
                vuMarkSensing = new VuMarkSensing(op.hardwareMap);
            } catch (Exception e) {
                RobotLog.e("missing: VuMark Sensing" + e.getMessage());
                numberMissing++;
            }
        }
        this.relicRecoveryVuMark = vuMarkSensing;

        HwGyro gyroSensor = null;
            try {
                gyroSensor = new HwBnoGyro(hardwareMap,
                                           "imu");
                while (gyroSensor.isCalibrating()) {
                    telemetry.addData("Gyro", "Calibrating");
                    telemetry.update();
                    Thread.sleep(50);
                }
            } catch (Exception e) {
                RobotLog.e("missing: gyro_sensor " + e.getMessage());
                numberMissing++;
            }
        this.gyroSensor = gyroSensor;

        this.drive = createDrive();

        telemetry.addData("Missing Devices", numberMissing);
        telemetry.update();

    }


    /**
     * RobotOvercharged and sensor shut down
     */
    public void close ()
    {
    }

    /*
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

    /*
     * update LEDs
     */

    public void drawLed () {
        for (HwLed led: leds) {
            led.draw();
        }
    }

}
