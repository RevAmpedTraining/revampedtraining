package com.revAmped.test;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * The name of this OpMode sounds like the name of a Greek foodcart critic
 */
public class GyroTest extends LinearOpMode {
    private final static int LED_GREEN = 1, LED_WHITE = 2; //digital
    private final static int SWITCH_SLIDE_DOWN_LEFT = 5, SWITCH_SLIDE_DOWN_RIGHT = 4, SWITCH_SLIDE_UP_LEFT = 6, SWITCH_SLIDE_UP_RIGHT = 7, LED_RED = 3, LED_BLUE = 2; //analog

    private final static float SERVO_BUCKET_TURN_CENTER = 0.485f, SERVO_BUCKET_TURN_LEFT = 0.685f, SERVO_BUCKET_TURN_RIGHT = 0.285f;
    private final static float SERVO_BUCKET_TURN2_CENTER = 0.295f, SERVO_BUCKET_TURN2_RIGHT = 0.095f, SERVO_BUCKET_TURN2_LEFT = 0.495f;
    private final static float SERVO_DOOR_CLOSED = 0.3f, SERVO_DOOR_OPEN = 0.6f;
    private final static float SERVO_HOOKR_UP = 1.0f, SERVO_HOOKR_DOWN = 0.07f;
    private final static float SERVO_HOOKL_UP = 0.0f, SERVO_HOOKL_DOWN = 0.94f;
    private final static float SERVO_SLIDE_START = 0.0f, SERVO_SLIDE_END = 0.68f;
    private final static float SERVO_CLIMBER_IN = 0.97f, SERVO_CLIMBER_OUT = 0.0f;
    private final static float SERVO_WINGL_IN = 0.96f, SERVO_WINGL_OUT = 0.35f;
    private final static float SERVO_WINGR_IN = 0.025f, SERVO_WINGR_OUT = 0.625f;
    private final static float HIGH_BUCKET_X = 34.0f, HIGH_BUCKET_Y = 17.0f;

    int  m_lastEncoder, m_lastTimeStamp,  initHeading;
    long initTimestamp, g_timestamp;

    DeviceInterfaceModule dim;
    ColorSensor colorSensor;
    ModernRoboticsI2cGyro gyroSensor;
    DcMotor driveLeftFront, driveLeftBack, driveRightFront, driveRightBack, slideLeft, slideRight, roller;
    Servo servoBucketTurn, servoDoor, servoHookLeft, servoHookRight, servoSlide, servoBucketTurn2, servoClimber, servoWingLeft, servoWingRight;
    int driveEncoderRF, driveEncoderLF, driveEncoderRB, driveEncoderLB;
    //int red, blue, green, clear;
    float distanceFromBeacon = 5.5f; //inches

    public boolean isRed = true;

    public void initialize() throws InterruptedException {
        hardwareMap.logDevices();
        try {
            dim = hardwareMap.deviceInterfaceModule.get("Device Interface Module");
        } catch (Exception e)
        {
            DbgLog.error("missing: " + "device interface module " + e.getMessage());
            dim = null;
        }

        try {
            driveLeftFront = hardwareMap.dcMotor.get("motor_lf");
            driveLeftFront.setDirection(DcMotor.Direction.FORWARD);
            driveLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            DbgLog.error("missing: " + "motor_lf " + e.getMessage());
            driveLeftFront = null;
        }
        try {
            driveLeftBack = hardwareMap.dcMotor.get("motor_lb");
            driveLeftBack.setDirection(DcMotor.Direction.FORWARD);
            driveLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            DbgLog.error("missing: " + "motor_lb " + e.getMessage());
            driveLeftBack = null;
        }
        try {
            driveRightFront = hardwareMap.dcMotor.get("motor_rf");
            driveRightFront.setDirection(DcMotor.Direction.REVERSE);
            driveRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e) {
            DbgLog.error("missing: " + "motor_rf " + e.getMessage());
            driveRightFront = null;
        }
        try {
            driveRightBack = hardwareMap.dcMotor.get("motor_rb");
            driveRightBack.setDirection(DcMotor.Direction.REVERSE);
            driveRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } catch (Exception e)
        {
            DbgLog.error("missing: " + "motor_rb " + e.getMessage());
            driveRightBack = null;
        }
        try {
            slideLeft = hardwareMap.dcMotor.get("motor_sl");
            slideLeft.setDirection(DcMotor.Direction.REVERSE);
            slideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e)
        {
            DbgLog.error("missing: " + "motor_sl " + e.getMessage());
            slideLeft = null;
        }
        try {
            slideRight = hardwareMap.dcMotor.get("motor_sr");
            slideRight.setDirection(DcMotor.Direction.FORWARD);
            slideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e)
        {
            DbgLog.error("missing: " + "motor_sr " + e.getMessage());
            slideRight = null;
        }
        try {
            roller = hardwareMap.dcMotor.get("motor_ro");
            roller.setDirection(DcMotor.Direction.REVERSE);
            roller.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e)
        {
            DbgLog.error("missing: " + "motor_ro " + e.getMessage());
            roller = null;
        }

        try {
            servoDoor = hardwareMap.servo.get("servo_door");
        } catch (Exception e)
        {
            DbgLog.error("missing: " + "servo_door " + e.getMessage());
            servoDoor = null;
        }

        try {
            servoBucketTurn = hardwareMap.servo.get("servo_bturn");
        } catch (Exception e)
        {
            DbgLog.error("missing: " + "servo_bturn " + e.getMessage());
            servoBucketTurn = null;
        }

        try {
            servoHookLeft = hardwareMap.servo.get("servo_hookl");
        } catch (Exception e)
        {
            DbgLog.error("missing: " + "servo_hookl " + e.getMessage());
            servoHookLeft = null;
        }

        try {
            servoHookRight = hardwareMap.servo.get("servo_hookr");
        } catch (Exception e)
        {
            DbgLog.error("missing: " + "servo_hookr " + e.getMessage());
            servoHookRight = null;
        }

        try {
            servoSlide = hardwareMap.servo.get("servo_slide");
        } catch (Exception e)
        {
            DbgLog.error("missing: " + "servo_slide " + e.getMessage());
            servoSlide = null;
        }

        try {
            servoWingLeft = hardwareMap.servo.get("servo_wingl");
        } catch (Exception e) {
            DbgLog.error("missing: " + "servo_wingl " + e.getMessage());
            servoWingLeft = null;
        }

        try {
            servoWingRight = hardwareMap.servo.get("servo_wingr");
        } catch (Exception e) {
            DbgLog.error("missing: " + "servo_wingr " + e.getMessage());
            servoWingRight = null;
        }

        try {
            colorSensor = hardwareMap.colorSensor.get("color_sensor");
        } catch (Exception e) {
            DbgLog.error("missing: " + "color_sensor " + e.getMessage());
            colorSensor = null;
        }

        try {
            gyroSensor = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro_sensor");
        } catch (Exception e) {
            DbgLog.error("missing: " + "gyro_sensor " + e.getMessage());
            gyroSensor = null;
        }

        if(servoBucketTurn != null) {
            servoBucketTurn.setPosition(SERVO_BUCKET_TURN_CENTER);
        }
        if(servoBucketTurn2 != null) {
            servoBucketTurn2.setPosition(SERVO_BUCKET_TURN2_CENTER);
        }
        if(servoDoor != null) {
            servoDoor.setPosition(SERVO_DOOR_OPEN);
        }
        if(servoHookLeft != null && servoHookRight != null) {
            servoHookLeft.setPosition(SERVO_HOOKL_UP);
            servoHookRight.setPosition(SERVO_HOOKR_UP);
        }
        if(servoClimber != null) {
            servoClimber.setPosition(SERVO_CLIMBER_IN);
        }
        if(servoSlide != null) {
            servoSlide.setPosition(SERVO_SLIDE_START);
        }
        if(servoWingLeft != null) {
            servoWingLeft.setPosition(SERVO_WINGL_IN);
        }
        if(servoWingRight != null) {
            servoWingRight.setPosition(SERVO_WINGR_IN);
        }

        if(colorSensor != null) {
            colorSensor.enableLed(true);
        }

        if(gyroSensor != null) {
            gyroSensor.resetZAxisIntegrator();
            gyroSensor.calibrate();
            while(gyroSensor.isCalibrating()) {
                Thread.sleep(20);
            }
        }

        if(dim != null) {
            dim.setDigitalChannelMode(LED_GREEN, DigitalChannelController.Mode.OUTPUT);
            dim.setDigitalChannelMode(LED_WHITE, DigitalChannelController.Mode.OUTPUT);
        }

        telemetry.addData("Ready", "GyroTest");
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        // Wait for the start button to be pressed
        waitForStart();
        telemetry.addData("starting", "");
        telemetry.addData("left front", driveEncoderLF);
        telemetry.addData("right front", driveEncoderRF);
        telemetry.addData("left back", driveEncoderLB);
        telemetry.addData("right back", driveEncoderRB);
        if(servoClimber != null) {
            servoClimber.setPosition(SERVO_CLIMBER_IN);
        }

        gyroTurn(180, 0.73f, 10000);
        wait(2000);
        telemetry.addData("done","");
        //gyroTurn(180, 0.73f, 10000); //positive is right, negative is left
        //gyroTurn(-180, 0.73f, 10000); //positive is right, negative is left
    }
    public void driveTest()throws InterruptedException {
        initTimestamp =  System.currentTimeMillis();
        g_timestamp =  System.currentTimeMillis();
        while(timeoutMillis(g_timestamp) < 10000) {
            g_timestamp = System.currentTimeMillis();
            if(driveLeftFront != null)
            {
                driveLeftFront.setPower(0.24);
                telemetry.addData("left front", driveLeftFront.getCurrentPosition());
            }

            if(driveRightFront != null)
            {
                driveRightFront.setPower(0.24);
                telemetry.addData("right front", driveRightFront.getCurrentPosition());
            }

            if(driveLeftBack != null)
            {
                driveLeftBack.setPower(0.24);
                telemetry.addData("left back", driveLeftBack.getCurrentPosition());
            }

            if(driveRightBack != null)
            {
                driveRightBack.setPower(0.24);
                telemetry.addData("right back", driveRightBack.getCurrentPosition());
            }

            idle();
        }
        stopDrive();
    }

    public float getHeading(float initHeading) {
        return gyroSensor.getIntegratedZValue() - initHeading;
    }

    public void gyroTurnSimple(int degrees, float power, int timeoutMil) throws InterruptedException {
        float turnPower;
        float initHeading = (float)gyroSensor.getIntegratedZValue();
        double heading = initHeading;
        initTimestamp = System.currentTimeMillis();
        g_timestamp =  System.currentTimeMillis();
        turnPower = power*(degrees > 0 ? 1 : -1);
        telemetry.addData("Timeout", timeoutMillis(g_timestamp) < timeoutMil);
        if(timeoutMillis(g_timestamp) < timeoutMil) telemetry.addData("Timedout", "");
        telemetry.addData("Heading", getHeading(initHeading));

        while (Math.abs(heading) < Math.abs(degrees) && timeoutMillis(g_timestamp) < timeoutMil) {
            g_timestamp =  System.currentTimeMillis();
            heading = getHeading(initHeading);

            driveLeftFront.setPower(turnPower);
            driveLeftBack.setPower(turnPower);
            driveRightFront.setPower(-turnPower);
            driveRightBack.setPower(-turnPower);

            telemetry.addData("Heading", getHeading(initHeading));
            telemetry.addData("Heading Reset", gyroSensor.getHeading());
            telemetry.addData("Power", turnPower);
            telemetry.addData("Reached", Math.abs(heading) < Math.abs(degrees));
            telemetry.addData("Timeout", timeoutMillis(g_timestamp) < timeoutMil);
            if(timeoutMillis(g_timestamp) < timeoutMil) telemetry.addData("Timedout", "");
            telemetry.addData("Time", timeoutMillis(g_timestamp));
            DbgLog.msg("heading" + heading);


            idle();
        }
        stopDrive();
    }

    public void gyroTurn(int degrees, float maxPower, int timeoutMil) throws InterruptedException {
        float turnPower;
        float initHeading = (float)gyroSensor.getIntegratedZValue();
        double heading = initHeading;
        initTimestamp =  System.currentTimeMillis();
        g_timestamp =  System.currentTimeMillis();
        maxPower = Math.abs(maxPower);
        if (maxPower > 0.73) {
            maxPower = 0.73f;
        }
        float target = (float) ((degrees % 360));
        float turn_up = (float) 0.02; // 0.02 per degree
        float turn_down = (float) 0.01667; //  0.01667 per degree
        //    FOR WHAT
        float threshold = 1.0f;
        float degreesLeft;

        while (Math.abs(heading) < Math.abs(degrees)&& timeoutMillis(g_timestamp) < timeoutMil) {
            g_timestamp =  System.currentTimeMillis();
            heading = getHeading(initHeading);
            degreesLeft = (float)(target - heading);
            telemetry.addData("degLeft", degreesLeft);

            if(Math.abs(heading) < Math.abs((float)(target*0.33))) { //ramp up for the first third
                turnPower = 0.73f;
                //turnPower = (float)(target > 0 ?  turn_up*Math.abs(heading) + 0.35 : -turn_up*Math.abs(heading) + 0.35); //start the minimum power at 0.2
            }
            else { //ramp down for the last two-thirds
                turnPower = (degreesLeft > 0 ? 0.73f : -0.73f) - (float)(turn_down*(target*0.66 - degreesLeft)); //decrease to a minimum of 0.3
                if(degreesLeft < 0) turnPower = Range.clip(turnPower, -1f, -0.3f);
                else if(degreesLeft > 0) turnPower = Range.clip(turnPower, 0.3f, 1f);
            }

            turnPower = Range.clip(turnPower, -maxPower, maxPower);
            if (Math.abs(degreesLeft) < threshold) {
                break;
            }

            driveLeftFront.setPower(turnPower);
            driveLeftBack.setPower(turnPower);
            driveRightFront.setPower(-turnPower);
            driveRightBack.setPower(-turnPower);

            telemetry.addData("Heading", getHeading(initHeading));
            telemetry.addData("Heading Reset", heading);
            telemetry.addData("Degrees Left", degreesLeft);
            telemetry.addData("Power", turnPower);
            telemetry.addData("Time", timeoutMillis(g_timestamp));
            DbgLog.msg("heading" + heading);
            DbgLog.msg("degrees left" + degreesLeft);
            DbgLog.msg("power" + turnPower);

            Thread.sleep(20);
            idle();
        }
        stopDrive();
    }

    public long timeoutMillis(long time) {
        return time - initTimestamp;
    }

    public void stopDrive() {
        if(driveLeftFront != null)
            driveLeftFront.setPower(0);
        if(driveRightFront != null)
            driveRightFront.setPower(0);
        if(driveLeftBack != null)
            driveLeftBack.setPower(0);
        if(driveRightBack != null)
            driveRightBack.setPower(0);
    }
}


