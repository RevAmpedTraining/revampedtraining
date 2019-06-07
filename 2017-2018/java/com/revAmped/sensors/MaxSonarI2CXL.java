package com.revAmped.sensors;

import com.qualcomm.robotcore.hardware.I2cAddr;

/**
 * Interface API to the I2CXL-MaxSonar-EZ ultrasonic ranger series
 *
 * @see <a href="http://www.maxbotix.com/articles/095.htm">http://www.maxbotix.com/articles/095.htm</a>
 */
public interface MaxSonarI2CXL
{
    /** MaxSonar 8 bits address */
    byte bDEFAULT_ADDRESS = (byte) 0xE0;

    byte bSTART_RANGING = (byte) 0x51;

    byte bDATA_LENGTH = (byte) 0x2;

    /** It is best to allow 100ms between readings to allow
     * for proper acoustic dissipation. */
    long RANGING_WAIT_MS = 100;

    /** Users requiring real-time information should command a range
     * reading ~80ms before reading the sensor. */
    long READING_WAIT_MS = 80;

    /**
     * Send an ultrasonic pulse to measure the distance
     */
    void startRanging ();

    /**
     * Send an ultrasonic pulse to measure the distance
     * and return the distance measured
     * @return the distance measured.  Returns 0 if there is an error.
     */
    int getDistance ();

    /**
     * Change I2C address to new address
     * @param newAddr new address
     */
    void changeAddress (I2cAddr newAddr);
}
