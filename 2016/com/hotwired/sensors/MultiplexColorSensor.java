package com.revAmped.sensors;

import android.graphics.Color;

import com.revAmped.config.RobotConstants.COLOR_SENSOR;
import com.revAmped.util.HwLog;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.Range;

import static com.qualcomm.robotcore.hardware.I2cWaitControl.WRITTEN;

/**
 * Created by zwang on 11/20/2016.
 */

public class MultiplexColorSensor {

    /** luminance when seeing a line */
    public final static int LUX_LINE_WHITE = 50;

    // Registers
    private static final int ENABLE = 0x80;
    private static final int ATIME = 0x81;
    private static final int CONTROL = 0x8F;
    private static final int ID = 0x92;
    private static final int STATUS = 0x93;
    private static final int CDATAL = 0x94;

    // Default I2C address for multiplexer. The address can be changed to any
    // value from 0x70 to 0x77, so this line would need to be changed if a
    // non-default address is to be used.
    public static final I2cAddr MUX_ADDRESS_0 = new I2cAddr(0x70);
    public static final I2cAddr MUX_ADDRESS_1 = new I2cAddr(0x71);
    public static final I2cAddr MUX_ADDRESS_2 = new I2cAddr(0x72);
    public static final I2cAddr MUX_ADDRESS_3 = new I2cAddr(0x73);
    public static final I2cAddr MUX_ADDRESS_4 = new I2cAddr(0x74);
    public static final I2cAddr MUX_ADDRESS_5 = new I2cAddr(0x75);
    public static final I2cAddr MUX_ADDRESS_6 = new I2cAddr(0x76);
    public static final I2cAddr MUX_ADDRESS_7 = new I2cAddr(0x77);

    private I2cDevice mux;
    private I2cDeviceSynch muxReader;

    // I2C address for color sensor
    private static final I2cAddr ADA_ADDRESS = new I2cAddr(0x29);
    private I2cDevice ada;
    private I2cDeviceSynch adaReader;

    private COLOR_SENSOR[] colorSensors;

    public static int GAIN_1X =  0x00;
    public static int GAIN_4X =  0x01;
    public static int GAIN_16X = 0x02;
    public static int GAIN_60X = 0x03;

    public static int INTEGRATION_TIME_2_4MS = 0xFF; /**< 2.4ms */
    public static int INTEGRATION_TIME_24MS = 0xF6; /**< 24ms */
    public static int INTEGRATION_TIME_50MS = 0xEB; /**< 50ms */
    public static int INTEGRATION_TIME_101MS = 0xD5; /**< 101ms */
    public static int INTEGRATION_TIME_154MS = 0xC0; /**< 154ms */
    public static int INTEGRATION_TIME_700MS = 0x00; /**< 700ms */

    public static final float SCALE_ADAFRUIT = 255.0f/400.0f;

    /**
     * Initializes Adafruit color sensors on the specified ports of the I2C
     * multiplexer.
     *
     * @param hardwareMap  hardwareMap from OpMode
     * @param muxName      Configuration name of I2CDevice for multiplexer
     * @param colorName    Configuration name of I2CDevice for color sensor
     * @param colorSensors        Out ports on multiplexer with color sensors attached
     */
    public MultiplexColorSensor(HardwareMap hardwareMap,
                                String muxName,
                                String colorName,
                                I2cAddr muxAddress,
                                COLOR_SENSOR[] colorSensors) {
        this.colorSensors = colorSensors;

        mux = hardwareMap.i2cDevice.get(muxName);
        muxReader = new I2cDeviceSynchImpl(mux, muxAddress, false);
        muxReader.engage();

        // Loop over the ports activating each color sensor
        for (int i = 0; i < this.colorSensors.length; i++) {
            // Write to given output port on the multiplexer
            muxReader.write8(0x0, 1 << this.colorSensors[i].channel, WRITTEN);

            ada = hardwareMap.i2cDevice.get(colorName);
            adaReader = new I2cDeviceSynchImpl(ada, ADA_ADDRESS, false);
            adaReader.engage();

            adaReader.write8(ENABLE, 0x03, WRITTEN);  // Power on and enable ADC
            adaReader.read8(ID);                   // Read device ID
            adaReader.write8(CONTROL, GAIN_4X, WRITTEN); // Set gain
            adaReader.write8(ATIME, INTEGRATION_TIME_24MS, WRITTEN);   // Set integration time
        }
    }

    public COLOR_SENSOR[] getColorSensors() {
        return colorSensors;
    }

    /**
     * Retrieve the color read by the given color sensor
     *
     * @param sensor Port on multiplexer of given color sensor
     * @return Array containing the Clear, Red, Green, and Blue color values
     */
    public synchronized int[] getCRGB(COLOR_SENSOR sensor) {
        // Write to I2C port on the multiplexer
        muxReader.write8(0x0, 1 << sensor.channel, WRITTEN);

        // Read color registers
        byte[] adaCache = adaReader.read(CDATAL, 8);

        // Combine high and low bytes
        int[] crgb = new int[4];
        for (int i=0; i<4; i++) {
            crgb[i] = (adaCache[2*i] & 0xFF) + (adaCache[2*i+1] & 0xFF) * 256;
        }

        // remove IR
        int ir = (crgb[1] + crgb[2] + crgb[3] > crgb[0]) ? (crgb[1] + crgb[2] + crgb[3] - crgb[0]) / 2 : 0;
        crgb[1] = crgb[1] - ir;
        crgb[2] = crgb[2] - ir;
        crgb[3] = crgb[3] - ir;
        crgb[0] = crgb[0] - ir;

        return crgb;
    }

    public int luminance(COLOR_SENSOR sensor) {
        int[] crgb = getCRGB(sensor);
        return (int)luminance(crgb);
    }

    public static int luminance(int[] crgb) {
        return Math.abs((short)((-0.32466F * crgb[1]) + (1.57837F * crgb[2]) + (-0.73191F * crgb[3])));
    }

    public int colorTemperature(int[] crgb) {
  /* 1. Map RGB values to their XYZ counterparts.    */
  /* Based on 6500K fluorescent, 3000K fluorescent   */
  /* and 60W incandescent values for a wide range.   */
  /* Note: Y = Illuminance or lux                    */
        float X = (-0.14282F * crgb[1]) + (1.54924F * crgb[2]) + (-0.95641F * crgb[3]);
        float Y = (-0.32466F * crgb[1]) + (1.57837F * crgb[2]) + (-0.73191F * crgb[3]);
        float Z = (-0.68202F * crgb[1]) + (0.77073F * crgb[2]) + (0.56332F * crgb[3]);

  /* 2. Calculate the chromaticity co-ordinates      */
        float xc = (X) / (X + Y + Z);
        float yc = (Y) / (X + Y + Z);

  /* 3. Use McCamy's formula to determine the CCT    */
        float n = (xc - 0.3320F) / (0.1858F - yc);

  /* Calculate the final CCT */
        float cct = (float)((449.0F * Math.pow(n, 3)) + (3525.0F * Math.pow(n, 2)) + (6823.3F * n) + 5520.33F);
        return (int)cct;
    }

    /**
     * get hue value from RGB values
     * copied from MRRGBExample.java
     * @return hue value
     */
    public float[] hsvValues(COLOR_SENSOR sensor) {
        int[] crgb = getCRGB(sensor);
        return hsvValues(crgb);
    }

    public static float[] hsvValues(int[] crgb) {
        float max = Math.max(Math.max(crgb[1], crgb[2]), crgb[3]);
        // auto scale
        if (max > 255.0f) {
            float scale = 255.0f/max;
            crgb[1] = Range.clip((int)(crgb[1]*scale), 0, 1);
            crgb[2] = Range.clip((int)(crgb[2]*scale), 0, 1);
            crgb[3] = Range.clip((int)(crgb[3]*scale), 0, 1);
        }

        float hsvValues[] = {0F,0F,0F};
        Color.RGBToHSV(crgb[1],
                       crgb[2],
                       crgb[3],
                       hsvValues);
        return hsvValues;
    }

    /**
     * get hue value from RGB values
     * copied from MRRGBExample.java
     * @return hue value
     */
    public float hue(COLOR_SENSOR port) {
        return hsvValues(port)[0];
    }

    /**
     * whether detects blue
     * @return whether detects blue
     */
    public static boolean isBlue (int[] crgb) {
        float[] hsv = hsvValues(crgb);
        int lux = luminance(crgb);
        return isBlue(hsv,
                      lux);
    }

    /**
     * whether detects blue
     * @return whether detects blue
     */
    public static boolean isBlue (float[] hsv,
                                  int lux) {
        return /*(lux > 2) &&*/ (205 <= hsv[0]) && (hsv[0] <= 270);
    }

    /**
     * whether detects red
     * @return whether detects red
     */
    public static boolean isRed (int[] crgb) {
        float[] hsv = hsvValues(crgb);
        int lux = luminance(crgb);
        return isRed(hsv,
                     lux);
    }

    /**
     * whether detects red
     * @return whether detects red
     */
    public static boolean isRed (float[] hsv,
                                 int lux) {
        return /*(lux > 2) &&*/ ((345 <= hsv[0] && hsv[0] <= 360) || (0 <= hsv[0] && hsv[0] <= 15));
    }

    public static Boolean isRed(int[] crgbF,
                                int[] crgbB) {
        float[] hsvF = hsvValues(crgbF);
        int luxF = luminance(crgbF);
        float[] hsvB = hsvValues(crgbB);
        int luxB = luminance(crgbB);

        Boolean isRed = null;
        if (isRed(hsvF, luxF)) {
            isRed = Boolean.TRUE;
        }
        else if (isBlue(hsvF, luxF)) {
            isRed = Boolean.FALSE;
        }
        else if (isRed(hsvB, luxB)) {
            isRed = Boolean.TRUE;
        }
        else if (isBlue(hsvB, luxB)) {
            isRed = Boolean.FALSE;
        }

        HwLog.i("Color isRed: " + isRed + " Front hue: " + hsvF[0] + " lux: " + luxF + " Back hue: " + hsvB[0] + " lux: " + luxB);
        return isRed;
    }
}
