package com.revAmped.sensors;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by swang4
 * 11/22/2017.
 */

public class RevColorDistanceSensor {

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;  // not used so far.

    HardwareMap hardwareMap;
    String sensorName;
    String sensorNameDistance;
    float  MULTIPLIER = 1.4f;   // if seeing one color's RGB value is   MULTIPLIER  the other color, it is the color of larger value
    //hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    public enum COLORTYPE
    {
        BLUE,
        RED,
        WHITE,
        NONE;
    }

    public RevColorDistanceSensor(HardwareMap hardwareMap, String strName) {

        this.hardwareMap = hardwareMap;
        sensorName = strName;
        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, sensorName);
        sensorNameDistance = strName;
        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, sensorNameDistance);


    }
    public double GetDistance() {
        double distance = 0;
        distance = sensorDistance.getDistance(DistanceUnit.CM);
        return distance;
    }
    public boolean isWhite() {
        float white = sensorColor.alpha();
        if (white > 30) {
            return true;
        }
        return false;
    }
    public COLORTYPE getColor()
    {

        // hsvValues is an array that will hold the hue, saturation, and value information.
        //float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        //int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        //final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        //while (opModeIsActive()) {
            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);


            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            /*relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });*/

        // Set the panel back to the default color
        /*relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });*/

        RobotLog.vv("RevColorSensor-Red", "%d", sensorColor.red());
        RobotLog.vv("RevColorSensor-Blue", "%d", sensorColor.blue());
        RobotLog.vv("RevColorSensor-Green", "%d", sensorColor.green());
        RobotLog.vv("RevColorSensor-hsv", "%f", hsvValues[0]);

        //12/16/2017: found the red value sometimes is too low. add the threashold 15 to make it is good reading.
        if (sensorColor.red() > 15 && sensorColor.red() > sensorColor.blue() * MULTIPLIER)
        {
            return COLORTYPE.RED;
        }
        else if (sensorColor.blue() > 15 && sensorColor.blue() > sensorColor.red() * MULTIPLIER)
        {
            return COLORTYPE.BLUE;
        }
        else if (sensorColor.blue() < 15 && sensorColor.blue() > sensorColor.red() *1.6f) {
            return COLORTYPE.BLUE;
        }else if (sensorColor.red() < 15 && sensorColor.red() > sensorColor.blue() *1.6f) {
            return COLORTYPE.RED;
        } else {
            return COLORTYPE.NONE;
        }
    }
    public double getDistance()
    {
        return sensorDistance.getDistance(DistanceUnit.CM);
    }

    //should check if the value is blank.  basically,   getColor has to be called first, in order to get the values of the following.
    public int getRed()
    {
        return sensorColor.red();
    }
    public int getAlpha()
    {
        return sensorColor.alpha();
    }
    public int getBlue()
    {
        return sensorColor.blue();
    }
    public int getGreen()
    {
        return sensorColor.green();
    }
    public float getHue()
    {
        return hsvValues[0];
    }
}