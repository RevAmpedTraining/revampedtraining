package com.revAmped.components;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Created by zwang on 2/18/2016.
 */
public class HwColorSensor
    extends HwDevice
{
    ColorSensor colorSensor;

    /**
     * initializes the color sensor
     * @param hardwareMap HardwareMap to get the sensor from
     * @param id name of color sensor
     * @throws IllegalArgumentException exception
     */
    public HwColorSensor(HardwareMap hardwareMap,
                         String id)
        throws IllegalArgumentException
    {
        super(id);

        this.colorSensor = hardwareMap.colorSensor.get(id);
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
    public float hue() {
        float hsvValues[] = {0F,0F,0F};

        Color.RGBToHSV(8 * colorSensor.red(),
                       8 * colorSensor.green(),
                       8 * colorSensor.blue(),
                       hsvValues);
        return hsvValues[0];
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
        float hsvValues[] = {0F,0F,0F};

        Color.RGBToHSV(8 * colorSensor.red(),
                       8 * colorSensor.green(),
                       8 * colorSensor.blue(),
                       hsvValues);
        return (int) hsvValues[0] == hue && hsvValues[1] > 0.6f && hsvValues[2] > 0.015f;
    }
}
