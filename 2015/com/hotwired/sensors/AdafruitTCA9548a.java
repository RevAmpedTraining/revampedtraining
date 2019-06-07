package com.revAmped.sensors;

/**
 * Interface API to the Adafruit TCA9548A 1-to-8 I2C Multiplexer Breakout.
 * You can create an implementation of this interface for a given sensor using
 * @see <a href="https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout">
 *     https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout</a>
 * @see <a href="https://cdn-learn.adafruit.com/downloads/pdf/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout.pdf">
 *     https://cdn-learn.adafruit.com/downloads/pdf/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout.pdf</a>
 */
public interface AdafruitTCA9548a {

    /**
     * selects channels to be enabled
     * @param channels channels enabled
     */
    void select(CHANNEL... channels);

    /** predefined I2C addresses */
    enum TCA_ADDR {
        ADDR0(0x70),
        ADDR1(0x71),
        ADDR2(0x72),
        ADDR3(0x73),
        ADDR4(0x74),
        ADDR5(0x75),
        ADDR6(0x76),
        ADDR7(0x77);

        private byte bVal;
        TCA_ADDR (int i)      {
            bVal = (byte)(i<<1);
        }
        public byte getValue() {
            return this.bVal;
        }
    }

    /**
     * multiplex Channels
     */
    enum CHANNEL {
        C0(0),
        C1(1),
        C2(2),
        C3(3),
        C4(4),
        C5(5),
        C6(6),
        C7(7);

        private byte bVal;
        CHANNEL(int i)      {
            bVal = (byte)(1<<i);
        }
        public byte getValue() {
            return this.bVal;
        }
    }
}
