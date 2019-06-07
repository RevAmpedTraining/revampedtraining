package com.revAmped.components;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

/**
 * Created by zwang on 11/11/2017.
 */

public class HwNormalizedColorSensor
 extends HwDevice{

    private NormalizedColorSensor colorSensor;

    public HwNormalizedColorSensor(HardwareMap hardwareMap,
                                   String colorName) {
        super(colorName);
        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, colorName);
    }

    /**
     * Reads the colors from the sensor
     * @return the current set of colors from the sensor
     */
    public NormalizedRGBA getNormalizedColors() {
        NormalizedRGBA nrgba = colorSensor.getNormalizedColors();

        // remove IR
        /*
        float ir = (nrgba.red + nrgba.green + nrgba.blue > nrgba.alpha) ? (nrgba.red + nrgba.green + nrgba.blue - nrgba.alpha) / 2f : 0f;
        nrgba.red = nrgba.red - ir;
        nrgba.green = nrgba.green - ir;
        nrgba.blue = nrgba.blue - ir;
        nrgba.alpha = nrgba.alpha - ir;
        */
        return nrgba;
    }

    /**
     * get hue value from RGB values
     * copied from MRRGBExample.java
     * @return hue value
     */
    public float[] hsvValues() {
        NormalizedRGBA nrgba = getNormalizedColors();
        return hsvValues(nrgba);
    }

    /**
     * get hue value from RGB values
     * copied from MRRGBExample.java
     * @return hue value
     */
    public static float[] hsvValues(NormalizedRGBA nrgba) {
        float hsvValues[] = {0F,0F,0F};
        Color.colorToHSV(nrgba.toColor(),
                         hsvValues);
        return hsvValues;
    }

    /**
     * get hue value from RGB values
     * copied from MRRGBExample.java
     * @return hue value
     */
    public float hue() {
        NormalizedRGBA nrgba = getNormalizedColors();
        return hue(nrgba);
    }

    /**
     * get hue value from RGB values
     * copied from MRRGBExample.java
     * @return hue value
     */
    public static float hue(NormalizedRGBA nrgba) {
        float hsvValues[] = hsvValues(nrgba);
        return hsvValues[0];
    }

    public float luminance() {
        NormalizedRGBA nrgba = getNormalizedColors();
        return luminance(nrgba);
    }

    // Digital ITU BT.601 (gives more weight to the R and B components)
    public static float luminance(NormalizedRGBA nrgba) {
        return (float)((0.299F * nrgba.red + 0.587F * nrgba.green + 0.114F * nrgba.blue)*255f);
}

    /**
     * whether detects blue
     * @return whether detects blue
     */
    public static boolean isBlue (float[] hsv,
                                  int lux) {
        return /*(lux > 2) &&*/ (170 <= hsv[0]) && (hsv[0] <= 270);
    }

    /**
     * whether detects red
     * @return whether detects red
     */
    public static boolean isRed (float[] hsv,
                                 int lux) {
        return /*(lux > 2) &&*/ ((345 <= hsv[0] && hsv[0] <= 360) || (0 <= hsv[0] && hsv[0] <= 15));
    }
}
