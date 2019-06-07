package com.revAmped.components;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by zwang on 2/18/2016.
 */
public class HwColorSensor
    extends HwDevice
{
    public static final float HUE_SCALE_HT = 1f;
    public static final float HUE_SCALE_MR = 8f;
    public static final float HUE_SCALE_ADAFRUIT = 255f/800f;

    protected ColorSensor colorSensor;

    // default is for MR color sensor
    private float scale = HUE_SCALE_MR;

    /**
     * initializes the color sensor
     * @param id name of color sensor
     * @throws IllegalArgumentException exception
     */
    public HwColorSensor(String id)
            throws IllegalArgumentException
    {
        super(id);
    }

    /**
     * initializes the color sensor
     * @param colorSensor color sensor
     * @param id name of color sensor
     * @throws IllegalArgumentException exception
     */
    public HwColorSensor(ColorSensor colorSensor,
                         String id)
        throws IllegalArgumentException
    {
        super(id);

        this.colorSensor = colorSensor;
    }

    public void setScale(float scale) {
        this.scale = scale;
    }

    public float getScale() {
        return this.scale;
    }

    /**
     * enable or disable the sensor LED
     * @param led if the sensor LED is enabled
     */
    public void enableLed(boolean led) {
        colorSensor.enableLed(led);
    }

    /**
     * get the red light sensor value
     * @return red sensor value
     */
    public int red() {
        return colorSensor.red();
    }

    /**
     * get the green light sensor value
     * @return green sensor value
     */
    public int green() {
        return colorSensor.green();
    }

    /**
     * get the blue light sensor value
     * @return blue sensor value
     */
    public int blue() {
        return colorSensor.blue();
    }

    /**
     * get the light sensor value
     * @return light sensor value
     */
    public int alpha() {
        return colorSensor.alpha();
    }

    /**
     * get the argb sensor value
     * @return argb sensor value
     */
    public int argb() {
        return colorSensor.argb();
    }

    /**
     * set the I2C address
     * @param address new I2C address
     */
    void setI2cAddress(I2cAddr address) {
        colorSensor.setI2cAddress(address);
    }

    /**
     * get the I2C address
     * @return current I2C address
     */
    I2cAddr getI2cAddress() {
        return colorSensor.getI2cAddress();
    }

    /**
     * get hue value from RGB values
     * copied from MRRGBExample.java
     * @return hue value
     */
    public float[] hueValues() {
        float hsvValues[] = {0F,0F,0F};

        int argb = colorSensor.argb();

        Color.RGBToHSV((int)(scale * ((argb >> 16) & 0xFF)),
                       (int)(scale * ((argb >> 8) & 0xFF)),
                       (int)(scale * (argb & 0xFF)),
                       hsvValues);
        return hsvValues;
    }

    /**
     * get hue value from RGB values
     * copied from MRRGBExample.java
     * @return hue value
     */
    public float hue() {
        return hueValues()[0];
    }

    /**
     * whether detects blue
     * @return whether detects blue
     */
    public boolean isBlue () {
        return isHue(240);
    }

    /**
     * whether detects red
     * @return whether detects red
     */
    public boolean isRed () {
        return isHue(0);
    }

    /**
     * whether detects green
     * @return whether detects green
     */
    public boolean isGreen () {
        return isHue(120);
    }

    private boolean isHue(int hue) {
        float[] hsvValues = hueValues();

        return (int) hsvValues[0] == hue && hsvValues[1] > 0.6f && hsvValues[2] > 0.015f;
    }
}
