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

import static com.revAmped.config.RobotOverchargedConstants.TAG_R;

/**
 * Overcharged Team #12599
 * Robot definition for TeleOp & Autonomous
 */
public class RobotOvercharged2
{
    protected Telemetry telemetry;
    public final VuMarkSensing relicRecoveryVuMark;
    public final List<HwDistanceSensor> distanceSensors = new ArrayList<>();

    ///Drive components
    public final HwServo servoLeftFront;
    public final HwServo servoLeftBack;
    public final HwServo servoRightFront;
    public final HwServo servoRightBack;
    public final HwMotor driveLeftFront;
    public final HwMotor driveLeftBack;
    public final HwMotor driveRightFront;
    public final HwMotor driveRightBack;
    public final Drive drive;

    ///Intake/Collection components
    public final HwMotor intakeLeft;
    public final HwMotor intakeRight;
    public final Intake intake;

    ///Slide components
    public final HwSwitch switchSlideUp;
    public final HwSwitch switchSlideDown;
    public final HwMotor slideMotor;

    ///Dumper components
    public final HwServo servoDumper;

    ///Robot Gyro sensor
    public final HwGyro gyroSensor;

    ///Jewel components
    public final HwServo servoJewel;
    public final HwServo servoJewelKnocker;

    public final List<HwServo> servos = new ArrayList<>();
    public final List<HwSwitch> switchs = new ArrayList<>();

    ///Led indicator components
    public final HwLed ledYellow;
    public final HwLed ledGreen;
    public final HwLed ledWhite;
    public final HwLed ledBlue;
    public final HwLed ledRed;
    public final List<HwLed> leds = new ArrayList<>();

    ///Relic Components
    public final HwSwitch switchRelicIn;
    public final HwMotor relicMotor;
    public final HwServo servoRelicElbow;
    public final HwServo servoRelicClaw;

    ///Sonar Sensors
    ///Sonar Sensor Left Front
    public final HwSonarAnalog sonarLF;
    ///Sonar Sensor Left Side
    public final HwSonarAnalog sonarLS;
    ///Sonar Sensor Right Back
    public final HwSonarAnalog sonarRB;
    public final List<HwSonarAnalog> sonars = new ArrayList<>();

    /**
     * initialize the robot
     * initialize all the hardware components used in our robot
     * @param op opMode to run
     * @param isAutonomous if autonomous
     */
    public RobotOvercharged2(OpMode op,
                             boolean isAutonomous)
    {
        ///report the number of missing components
        int numberMissing = 0;
        HardwareMap hardwareMap = op.hardwareMap;
        this.telemetry = op.telemetry;

        hardwareMap.logDevices();

        RobotLog.ee(TAG_R, "Initializing motors");
        ///Initialize Motors
        HwMotor driveLeftFront = null;
        try {
            driveLeftFront = new HwMotor(hardwareMap,
                    "driveLF",
                    DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            RobotLog.ee(TAG_R, "missing: driveLF " + e.getMessage());
            numberMissing++;
        }
        this.driveLeftFront = driveLeftFront;

        HwMotor driveLeftBack = null;
        try {
            driveLeftBack = new HwMotor(hardwareMap,
                    "driveLB",
                    DcMotor.Direction.FORWARD);
        } catch (Exception e) {
            RobotLog.ee(TAG_R, "missing: driveLB " + e.getMessage());
            numberMissing++;
        }
        this.driveLeftBack = driveLeftBack;

        HwMotor driveRightFront = null;
        try {
            driveRightFront = new HwMotor(hardwareMap,
                    "driveRF",
                    DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: driveRF " + e.getMessage());
            numberMissing++;
        }
        this.driveRightFront = driveRightFront;

        HwMotor driveRightBack = null;
        try {
            driveRightBack = new HwMotor(hardwareMap,
                    "driveRB",
                    DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: driveRB " + e.getMessage());
            numberMissing++;
        }
        this.driveRightBack = driveRightBack;

        HwMotor slideMotor = null;
        try {
            slideMotor = new HwMotor(hardwareMap,
                    "slideMotor",
                    DcMotor.Direction.REVERSE);
        } catch (Exception e)
        {
            RobotLog.ee(TAG_R,  "missing: slideMotor " + e.getMessage());
            numberMissing++;
        }
        this.slideMotor = slideMotor;

        HwMotor intakeLeft = null;
        try {
            intakeLeft = new HwMotor(hardwareMap,
                    "intakeL",
                    DcMotor.Direction.REVERSE,
                    DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e)
        {
            RobotLog.ee(TAG_R,  "missing: intakeL " + e.getMessage());
            numberMissing++;
        }
        this.intakeLeft = intakeLeft;

        HwMotor intakeRight = null;
        try {
            intakeRight = new HwMotor(hardwareMap,
                    "intakeR",
                    DcMotor.Direction.FORWARD,
                    DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e)
        {
            RobotLog.ee(TAG_R,  "missing: intakeR " + e.getMessage());
            numberMissing++;
        }
        this.intakeRight = intakeRight;
        ///create the intake component object
        this.intake = createIntake();

        HwMotor relicMotor = null;
        try {
            relicMotor = new HwMotor(hardwareMap,
                    "relicMotor",
                    DcMotor.Direction.FORWARD,
                    DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e)
        {
            RobotLog.ee(TAG_R,  "missing: relicMotor " + e.getMessage());
            numberMissing++;
        }
        this.relicMotor = relicMotor;

        RobotLog.ee(TAG_R,  "Initializing Servos");
        ///Initialize Servos
        HwServo servoRightFront = null;
        try {
            servoRightFront = new HwServo(hardwareMap,
                    "servoRF",
                    SwerveDriveConstants.SERVO_RIGHTFRONT_START);
            servos.add(servoRightFront);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: servoRF " + e.getMessage());
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
            RobotLog.ee(TAG_R,  "missing: servoRB " + e.getMessage());
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
            RobotLog.ee(TAG_R,  "missing: servoLF " + e.getMessage());
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
            RobotLog.ee(TAG_R,  "missing: servoLB " + e.getMessage());
            numberMissing++;
        }
        this.servoLeftBack = servoLeftBack;

        HwServo servoJewelKnocker = null;
        try {
            servoJewelKnocker = new HwServo(hardwareMap,
                    "servoJewelKnocker",
                    RobotOverchargedConstants.SERVO_JEWEL_KOCKER_MID);
            servos.add(servoJewelKnocker);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: servoJewelKnocker " + e.getMessage());
            numberMissing++;
        }
        this.servoJewelKnocker = servoJewelKnocker;

        HwServo servoJewel = null;
        try {
            servoJewel = new HwServo(hardwareMap,
                    "servoJewel",
                    RobotOverchargedConstants.SERVO_JEWEL_IN);
            servos.add(servoJewel);
        } catch (Exception e) {
            RobotLog.ee(TAG_R, "missing: servoJewel " + e.getMessage());
            numberMissing++;
        }
        this.servoJewel = servoJewel;

        HwServo servoDumper = null;
        try {
            servoDumper = new HwServo(hardwareMap,
                    "servoDumper",
                    RobotOverchargedConstants.SERVO_CONTAINER_NORMAL);
            servos.add(servoDumper);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: servoDumper " + e.getMessage());
            numberMissing++;
        }
        this.servoDumper = servoDumper;

        HwServo servoRelicElbow = null;
        try {
            servoRelicElbow = new HwServo(hardwareMap,
                    "relicElbow",
                    RobotOverchargedConstants.SERVO_RELIC_ELBOW_REST);
            servos.add(servoRelicElbow);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: relicElbow " + e.getMessage());
            numberMissing++;
        }
        this.servoRelicElbow = servoRelicElbow;

        HwServo servoRelicClaw = null;
        try {
            servoRelicClaw = new HwServo(hardwareMap,
                    "relicClaw",
                    RobotOverchargedConstants.SERVO_RELIC_CLAW_REST);
            servos.add(servoRelicClaw);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: relicClaw " + e.getMessage());
            numberMissing++;
        }
        this.servoRelicClaw = servoRelicClaw;

        RobotLog.ee(TAG_R,  "Initializing switches");
        HwSwitch hwSwitch = null;
        ///Initialize switches
        try {
            hwSwitch = new HwSwitch(hardwareMap,
                    "switch_slide_up",
                    true);
            switchs.add(hwSwitch);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: switch_slide_up " + e.getMessage());
            numberMissing++;
        }
        switchSlideUp = hwSwitch;
        hwSwitch = null;
        try {
            hwSwitch = new HwSwitch(hardwareMap,
                    "switch_slide_down",
                    true);
            switchs.add(hwSwitch);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: switch_slide_down " + e.getMessage());
            numberMissing++;
        }
        switchSlideDown = hwSwitch;

        hwSwitch = null;
        try {
            hwSwitch = new HwSwitch(hardwareMap,
                    "switch_relic_in",
                    true);
            switchs.add(hwSwitch);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: switch_relic " + e.getMessage());
            numberMissing++;
        }
        switchRelicIn = hwSwitch;

        RobotLog.ee(TAG_R,  "Initializing Leds");
        ///Initialize Leds
        HwLed led = null;
        try {
            led = new HwLed(hardwareMap,
                    "led_yellow");
            leds.add(led);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: led_yellow " + e.getMessage());
            numberMissing++;
        }
        ledYellow = led;
        led = null;
        try {
            led = new HwLed(hardwareMap,
                    "led_green");
            leds.add(led);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: led_green " + e.getMessage());
            numberMissing++;
        }
        ledGreen = led;
        led = null;
        try {
            led = new HwLed(hardwareMap,
                    "led_white");
            leds.add(led);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: led_white " + e.getMessage());
            numberMissing++;
        }
        ledWhite = led;
        led = null;
        try {
            led = new HwLed(hardwareMap,
                    "led_blue");
            leds.add(led);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: led_blue " + e.getMessage());
            numberMissing++;
        }
        ledBlue = led;
        led = null;
        try {
            led = new HwLed(hardwareMap,
                    "led_red");
            leds.add(led);
        } catch (Exception e) {
            RobotLog.ee(TAG_R,  "missing: led_red " + e.getMessage());
            numberMissing++;
        }
        ledRed = led;

        RobotLog.ii(TAG_R, "Initializing Gyro sensor");
        ///Initialize the Gyro sensor used in this robot
        HwGyro gyroSensor = null;
        try {
            gyroSensor = new HwNavGyro(hardwareMap,
                    "imu2");
            while (isAutonomous && gyroSensor.isCalibrating()) {
                telemetry.addData("Gyro", "Calibrating");
                telemetry.update();
                Thread.sleep(50);
            }
        } catch (Exception e) {
            RobotLog.ee(TAG_R, "missing: gyro_sensor " + e.getMessage());
            numberMissing++;
        }
        this.gyroSensor = gyroSensor;
        this.drive = createDrive();

        RobotLog.ii(TAG_R,  "Initializing Sonar sensor");
        ///Initialize the Sonar sensors used in this robot
        sonarLF = new HwSonarAnalog(hardwareMap,
                "sonarLF",
                HwSonarAnalog.SCALE_MAX_XL);
        sonars.add(sonarLF);
        sonarLS = new HwSonarAnalog(hardwareMap,
                "sonarLS",
                HwSonarAnalog.SCALE_MAX_XL);
        sonars.add(sonarLS);
        sonarRB = new HwSonarAnalog(hardwareMap,
                "sonarRB",
                HwSonarAnalog.SCALE_MAX_XL);
        sonars.add(sonarRB);

        VuMarkSensing vuMarkSensing = null;
        if (isAutonomous) {
            RobotLog.ii(TAG_R,  "Initializing VuMarkSensing");
            ///Initialize the VuMarkSensing for Autonomous only
            try {
                vuMarkSensing = new VuMarkSensing(op.hardwareMap);
            } catch (Exception e) {
                RobotLog.ee(TAG_R, "missing: VuMark Sensing" + e.getMessage());
                numberMissing++;
            }
        }
        this.relicRecoveryVuMark = vuMarkSensing;

        RobotLog.ii(TAG_R,  "Initializing done");
        telemetry.addData("Missing Devices", numberMissing);
        telemetry.update();

    }


    /**
     * RobotOvercharged and sensor shut down
     */
    public void close ()
    {
    }

    /**
     * subclass override this method
     * @return Drive
     */
    protected Intake createIntake () {
        return new Intake(intakeLeft, intakeRight, this);
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