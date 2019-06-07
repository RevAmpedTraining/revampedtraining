package com.revAmped.test;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
/**
 * Created by Justin on 1/1/2016.
 */
public class InverseKinematicsTest extends OpMode {
    private final static double l1 = 25.5, l2 = 21.5;

    private final static float SERVO_BUCKET_TURN_CENTER = (float)0.485, SERVO_BUCKET_TURN_LEFT = (float)0.685, SERVO_BUCKET_TURN_RIGHT = (float)0.285;
    //bucket left forward = 0.87, bucket right forward = 0.07
    private final static float SERVO_BUCKET_TURN2_CENTER = 0.3f, SERVO_BUCKET_TURN2_RIGHT = 0.1f, SERVO_BUCKET_TURN2_LEFT = 0.5f;
    public double bucketTurn1Angle, bucketTurn2Angle, x, y = -47;
    public float posBucketTurn1 = SERVO_BUCKET_TURN_CENTER, posBucketTurn2 = SERVO_BUCKET_TURN2_CENTER;
    public boolean control = false, left = false;
    public double maximum = Math.pow(l1+l2, 2);

    Servo servoBucketTurn, servoBucketTurn2;

    public int g_timestamp = 0;
    private final int BTN_PRESS_INTERVAL = 3; //3 tenths of a second
    public int[] lastButtonPress = new int[EButtonPressTimes.NUM_TIMED_BUTTONS.getValue()];

    public boolean CAN_PRESS(EButtonPressTimes x) {
        return Math.abs(g_timestamp - lastButtonPress[x.getValue()]) > BTN_PRESS_INTERVAL;
    }

    /**
     * enumeration for press intervals
     */
    public static enum EButtonPressTimes {
        BTN_PLUS(0),
        BTN_MINUS(1),
        BTN_PLUS2(2),
        BTN_MINUS2(3),
        BTN_PRESET_CENTER(4),
        BTN_PRESET_LEFT(5),
        BTN_PRESET_RIGHT(6),
        NUM_TIMED_BUTTONS(7);

        private final int value;
        EButtonPressTimes(int value) {
            this.value = value;
        }
        public int getValue() {return value;}
    }

    @Override
    public void init() {
        hardwareMap.logDevices();
        try {
            servoBucketTurn = hardwareMap.servo.get("servo_bturn");
        } catch (Exception e)
        {
            DbgLog.error("missing: " + "servo_bturn " + e.getMessage());
            servoBucketTurn = null;
        }

        try {
            servoBucketTurn2 = hardwareMap.servo.get("servo_bturn2");
        } catch (Exception e)
        {
            DbgLog.error("missing: " + "servo_bturn2 " + e.getMessage());
            servoBucketTurn2 = null;
        }

        if(servoBucketTurn != null && servoBucketTurn2 != null) {
            servoBucketTurn.setPosition(SERVO_BUCKET_TURN_CENTER);
            servoBucketTurn2.setPosition(SERVO_BUCKET_TURN2_CENTER);
        }

        telemetry.addData("Waiting", 0);
    }

    public boolean limit(double x, double y) {
        if(((Math.pow(x,2) + Math.pow(y,2)) > maximum) || (x == 0 && y == 0)) return true;
        return false;
    }

    @Override
    public void loop() {
        g_timestamp = (int) System.currentTimeMillis()/100;

        if(gamepad1.a && CAN_PRESS(EButtonPressTimes.BTN_MINUS)) {
            lastButtonPress[EButtonPressTimes.BTN_MINUS.getValue()] = g_timestamp;
            if(!limit(x,y-1)) y -= 1;
        }
        else if(gamepad1.y && CAN_PRESS(EButtonPressTimes.BTN_PLUS)) {
            lastButtonPress[EButtonPressTimes.BTN_PLUS.getValue()] = g_timestamp;
            if(!limit(x, y+1)) y += 1;
        }
        if(gamepad1.x && CAN_PRESS(EButtonPressTimes.BTN_MINUS2)) {
            lastButtonPress[EButtonPressTimes.BTN_MINUS2.getValue()] = g_timestamp;
            if(!limit(x-1, y)) x -= 1;
        }
        else if(gamepad1.b && CAN_PRESS(EButtonPressTimes.BTN_PLUS2)) {
            lastButtonPress[EButtonPressTimes.BTN_PLUS2.getValue()] = g_timestamp;
            if(!limit(x+1, y)) x += 1;
        }

        if(gamepad1.dpad_down && CAN_PRESS(EButtonPressTimes.BTN_PRESET_CENTER)) {
            lastButtonPress[EButtonPressTimes.BTN_PRESET_CENTER.getValue()] = g_timestamp;
            servoBucketTurn.setPosition(SERVO_BUCKET_TURN_CENTER);
            servoBucketTurn2.setPosition(SERVO_BUCKET_TURN2_CENTER);
            x = 0;
            y = -47;
            control = false;
        }
        else if(gamepad1.dpad_left && CAN_PRESS(EButtonPressTimes.BTN_PRESET_LEFT)) {
            lastButtonPress[EButtonPressTimes.BTN_PRESET_LEFT.getValue()] = g_timestamp;
            servoBucketTurn.setPosition(SERVO_BUCKET_TURN_LEFT);
            servoBucketTurn2.setPosition(SERVO_BUCKET_TURN2_CENTER);
            x = -47;
            y = 0;
            control = true;
            left = true;
        }
        else if(gamepad1.dpad_right && CAN_PRESS(EButtonPressTimes.BTN_PRESET_RIGHT)) {
            lastButtonPress[EButtonPressTimes.BTN_PRESET_RIGHT.getValue()] = g_timestamp;
            servoBucketTurn.setPosition(SERVO_BUCKET_TURN_RIGHT);
            servoBucketTurn2.setPosition(SERVO_BUCKET_TURN2_CENTER);
            x = 47;
            y = 0;
            control = true;
            left = false;
        }

        x = Range.clip(x, -(l1+l2), l1+l2);
        y = Range.clip(y, -(l1+l2), l1+l2);

        findAngles(x, y);
        degreesToTicks();

        posBucketTurn1 = Range.clip(posBucketTurn1, 0.085f, 0.885f);
        posBucketTurn2 = Range.clip(posBucketTurn2, 0, 1);

        if(control) {
            telemetry.addData("posBucketTurn1", posBucketTurn1);
            telemetry.addData("posBucketTurn2", posBucketTurn2);
            if(y != 47) {
                servoBucketTurn.setPosition(posBucketTurn1);
                servoBucketTurn2.setPosition(posBucketTurn2);
            }
            else {
                servoBucketTurn.setPosition(SERVO_BUCKET_TURN_CENTER);
                servoBucketTurn2.setPosition(SERVO_BUCKET_TURN2_CENTER);
            }
        }
    }

    public void findAngles(double x, double y) {
        double a, b, c, d;
        a = Math.pow(x, 2) + Math.pow((left ? -y : y), 2) - Math.pow(l1, 2) - Math.pow(l2, 2);
        a /= (2 * l1 * l2);
        if (1 - Math.pow(a, 2) < 0) {
            b = 0;
        } else {
            b = -1 * Math.sqrt(1 - Math.pow(a, 2));
        }
        bucketTurn2Angle = Math.atan2(b,a);

        c = x * (l1 + l2 * Math.cos(bucketTurn2Angle)) + (left ? -y : y) * l2 * Math.sin(bucketTurn2Angle);
        c /= Math.pow(x, 2) + Math.pow((left ? -y : y), 2);
        if (1 - Math.pow(c, 2) < 0) {
            d = 0;
        } else {
            d = Math.sqrt(1 - Math.pow(c, 2));
        }
        bucketTurn1Angle = Math.atan2(d,c);
        double bucketTurn1Angle1 = Math.atan2(d,c);
        double bucketTurn1Angle2 = Math.atan2(-d,c);

        double forwardx1=(l1 * Math.cos(bucketTurn1Angle1)) + (l2 * Math.cos(bucketTurn1Angle1 + bucketTurn2Angle));
        double forwardy1=(l1 * Math.sin(bucketTurn1Angle1)) + (l2 * Math.sin(bucketTurn1Angle1 + bucketTurn2Angle));

        double forwardx2=(l1 * Math.cos(bucketTurn1Angle2)) + (l2 * Math.cos(bucketTurn1Angle2 + bucketTurn2Angle));
        double forwardy2=(l1 * Math.sin(bucketTurn1Angle2)) + (l2 * Math.sin(bucketTurn1Angle2 + bucketTurn2Angle));

        telemetry.addData("x", x);
        telemetry.addData("y", y);

        DbgLog.msg(String.format("x: %s", x));
        DbgLog.msg(String.format("y: %s", y));

//        telemetry.addData("x1", forwardx1);
//        telemetry.addData("y1", forwardy1);
        DbgLog.msg(String.format("forwardx1: %s", forwardx1));
        DbgLog.msg(String.format("forwardy1: %s", forwardy1));

//        telemetry.addData("x2", forwardx2);
//        telemetry.addData("y2", forwardy2);
        DbgLog.msg(String.format("forwardx2: %s", forwardx2));
        DbgLog.msg(String.format("forwardy2: %s", forwardy2));

        double forwardx=(l1 * Math.cos(bucketTurn1Angle)) + (l2 * Math.cos(bucketTurn1Angle + bucketTurn2Angle));
        double forwardy=(l1 * Math.sin(bucketTurn1Angle)) + (l2 * Math.sin(bucketTurn1Angle + bucketTurn2Angle));

        if(Math.abs(x - forwardx) > 0.1 && Math.abs(y - forwardy) > 0.1) {
            bucketTurn1Angle = Math.atan2(-d,c);
            if(left) bucketTurn1Angle += (2*3.14159265);
        }

        bucketTurn1Angle = Math.toDegrees(bucketTurn1Angle);
        bucketTurn2Angle = Math.toDegrees(bucketTurn2Angle);
        telemetry.addData("bucketTurn1Angle", bucketTurn1Angle);
        telemetry.addData("bucketTurn2Angle", bucketTurn2Angle);
    }

    public void degreesToTicks() {
        float x1, y1;
        if(left) {
            x1 = -0.4f + (float)((0.002222222222) * (270-bucketTurn1Angle));
            y1 = (float) ((0.002222222222) * (-bucketTurn2Angle));
        }
        else {
            x1 = (float)((0.002222222222) * (bucketTurn1Angle+90));
            y1 = (float) ((0.002222222222) * (bucketTurn2Angle));
        }
        posBucketTurn1 = SERVO_BUCKET_TURN_CENTER -x1;
        posBucketTurn2 = SERVO_BUCKET_TURN2_CENTER -y1;
    }
}
