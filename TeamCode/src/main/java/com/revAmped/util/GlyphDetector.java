package com.revAmped.util;
import com.revAmped.components.HwSonarAnalog;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by swang4 on 1/25/2018.
 * utilize two color/distance sensors to detect
 * (1) how many glyphs are in the cart:
 *           distance_sensor_door_value  |   distance_sensor_back_value      |    Number_of_glyth
 *              > 0 and < 30             and      > 0 and < 10               |         2
 *                  > 30cm => (zero)     and      < 20cm                     |         1
 *                  < 30cm               and      > 20cm                     |         1
 *                  > 30cm               and      > 20cm                     |         0
 * (2) what is the color of glyph  in the cart
 *           read Hue 3 times.  and average.
 *           > 28   => brown
 *           < 28   =>  silver
 *
 * (3) distance toward the first glyph (analog input).
 */

public class GlyphDetector {

    public enum GlyphColor
    {
        BROWN,
        SILVER,
        NONE;    // means no color is valid.
    }
    public enum GlyphPos
    {
        DOOR_NEAR,
        DOOR_FAR,
        UNKNOWN;
    }
    //TODO: declare  members:   door and back sensor of type RevColorDistanceSensor
    //                    distance sensor of type ReVampDistanceSensor
    DistanceSensor sensor_door;
    DistanceSensor sensor_tray;
    HwSonarAnalog sensor_sonar_front;

    String str_sensor_door;
    String str_sensor_tray;

    // declare constant members:
    public final int GLYPH_HUE_THRESHOLD = 28;    // if hue > 28   brown,  if < 28, silver.
    public final int TRAY_GLYPH_DISTANCE = 20;    // if the distance is < THRESHOLD,  isSeen = true;   =>  close to the glyph, slow down.
    public final int DOOR_GLYPH_DISTANCE = 10;    // if the distance is < THRESHOLD,  isSeen = true;   =>  close to the glyph, slow down.
    //     otherwise,  isSeen is false  => continue acceleration.

    //constructor
    public GlyphDetector(HardwareMap hwMap, String sensor_Door, String sensor_Tray, String sonar_front)
    {
        // TODO: assign the members with input arguments, which was passed from Run .
        str_sensor_door = sensor_Door;
        str_sensor_tray = sensor_Tray;
        if (sensor_Door == "") {

        } else {
            sensor_door = hwMap.get(DistanceSensor.class, sensor_Door);

        }

        sensor_tray = hwMap.get(DistanceSensor.class, sensor_Tray);
        if (sonar_front=="") {

        } else {
            sensor_sonar_front = new HwSonarAnalog(hwMap, sonar_front, HwSonarAnalog.SCALE_MAX_XL);
        }
    }

    /*
    *  Get the number of Glyphs in the cart.
    */
    //detect jamming in teleop
    public boolean isJam() {
        boolean jam = false;
        if (sensor_door.getDistance(DistanceUnit.CM) < 100) {
            jam = true;
            RobotLog.vv("jam?", "TRUE");
        }
        return jam;
    }
    public boolean  IsTwoGlyphs()
    {
        boolean  ret = false;
        if (sensor_tray.getDistance(DistanceUnit.CM) < TRAY_GLYPH_DISTANCE) {
            ret = true;
            RobotLog.vv("IsTwoGlyph", "TRUE");
        }
        RobotLog.vv("IsTwoGlyph", "FALSE");

        return ret;
    }

    // if both color sensor do not see  the glyph,   or if
    // prev_count can be only 1 or zero.   if it is two, then no need to call.
    public int  GetGlyphCount(int prev_count)
    {
        //3/6/2018
        //only using tray sensor for super regionals
        int count = 0;
        if (sensor_tray.getDistance(DistanceUnit.CM) < TRAY_GLYPH_DISTANCE)
        {
            count= 2 ;
        }
        /*else if(sensor_door.getDistance(DistanceUnit.CM) <DOOR_GLYPH_DISTANCE)
        {
            count = 1;
        }*/
        /*
        int count = prev_count;
        switch (prev_count) {
            case 0:
                if (sensor_door.getDistance() > DOOR_GLYPH_DISTANCE )  //  Not SEEN Glyph by Door sensor, no need to check TRAY sensor
                    count = 0;
                else if (sensor_tray.getDistance() < TRAY_GLYPH_DISTANCE){   // TRAY sensor sees Glyph  in the back
                    count = 2;
                }
                else  // only the door sensor sees the glyph.
                {
                    count = 1;
                }
                break;
            case 1:
                if (sensor_tray.getDistance() < TRAY_GLYPH_DISTANCE)
                    count = 2;

                break;
        }*/
        RobotLog.vv("GlyphCount", "%d", count);
        return count;
    }


    /*
    * Input: the position of the Glyph in the cart (FRONT -  DOOR,    BACK -  FAR from the door)
    * output:  the color of the glyph detected.
    * */

    public GlyphColor GetGlythColor(GlyphPos pos)
    {
        GlyphColor color = GlyphColor.NONE;
        // TODO LIST,  add logic here to termine the color
        switch(pos) {
            case DOOR_NEAR:
                //TODO: read the hue of the color sensor on the door,   and check the value.
                break;
            case DOOR_FAR:
                //TODO: read the hue of the color sensor on the door,   and check the value.
                break;
            default:
                break;
        }
        return color;
    }

    /*
    * return the distance to the closest Glyph seen by the sonar sensor, unit of centimeters*/
    public float GetDistance(String sensor_name)
    {
        double distance = 0;
        // TODO List:  read the value of the sonor sensor, and convert it to centimeter.
        if (sensor_name == str_sensor_door)
        {
            distance = sensor_door.getDistance(DistanceUnit.CM);
            RobotLog.vv("DoorDistanceToGlyph", "%4.1f", distance);
        }
        else if (sensor_name == str_sensor_tray)
        {

            distance = sensor_tray.getDistance(DistanceUnit.CM);
            RobotLog.vv("TrayDistanceToGlyph", "%5.2f", distance);
        }
        else
        {

            distance = sensor_sonar_front.getDistance();
            RobotLog.vv("SonarDistanceToGlyph", "%5.1f", distance);
        }

        return (float) distance;
    }
}
