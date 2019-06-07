package com.revAmped.sensors;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

/**
 * Interface API to the Adafruit TCA9548A 1-to-8 I2C Multiplexer Breakout.
 * You can create an implementation of this interface for a given sensor using
 * @see <a href="https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout">
 *     https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout</a>
 * @see <a href="https://cdn-learn.adafruit.com/downloads/pdf/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout.pdf">
 *     https://cdn-learn.adafruit.com/downloads/pdf/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout.pdf</a>
 */
public class AdafruitTCA9548aMux
    extends HwI2cDevice
    implements AdafruitTCA9548a {

    public AdafruitTCA9548aMux(OpMode opmodeContext,
                               I2cDevice i2cDevice) {
        this(opmodeContext,
             i2cDevice,
             TCA_ADDR.ADDR0);
    }

    public AdafruitTCA9548aMux(OpMode opmodeContext,
                               I2cDevice i2cDevice,
                               TCA_ADDR addr) {
        super(opmodeContext,
              i2cDevice,
              new I2cAddr(addr.getValue()));

        initialize();
    }

    public AdafruitTCA9548aMux(OpMode opmodeContext,
                               I2cDeviceSynch deviceClient) {
        this(opmodeContext,
             deviceClient,
             TCA_ADDR.ADDR0);
    }

    public AdafruitTCA9548aMux(OpMode opmodeContext,
                               I2cDeviceSynch deviceClient,
                               TCA_ADDR addr) {
        super(opmodeContext,
              deviceClient,
              new I2cAddr(addr.getValue()));

        initialize();
    }

    @Override
    protected void initialize() {
        // We don't have the device auto-close since *we* handle the shutdown logic
        this.deviceClient.engage();

        setLoggingTag("TCA9548a");
    }

    @Override
    public void select(CHANNEL... channels) {

        byte channs = 0;
        for (CHANNEL channel : channels) {
            if (channel == null) {
                continue;
            }

            channs |= channel.getValue();
        }

        this.deviceClient.write8(channs, 0x0);
    }
}
