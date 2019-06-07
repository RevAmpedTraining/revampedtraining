package com.revAmped.components;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.revAmped.components.HwLed.ELedStatus.LED_BLINK;
import static com.revAmped.components.HwLed.ELedStatus.LED_OFF;
import static com.revAmped.components.HwLed.ELedStatus.LED_ON;

/**
 * Created by zwang status 2/16/2016.
 */
public class HwLed
    extends HwDevice {

    private DigitalChannel channel;

    private ELedStatus status = LED_OFF;

    private long blinkStartTime = System.currentTimeMillis();;
    private boolean isBlinkOn = true;

    public enum ELedStatus
    {
        LED_ON,
        LED_OFF,
        LED_BLINK;
    }

    private static final int BLINK_DURATION = 200;

    /**
     * initialize the LEDs
     * @param hardwareMap HardwareMap to get motor from
     * @param ledName name of switches
     */
    public HwLed(HardwareMap hardwareMap,
                 String ledName) {
        super(ledName);
        // get a reference to our LED object.
        channel = hardwareMap.get(DigitalChannel.class, ledName);

        // set the channel channel to input.
        channel.setMode(DigitalChannel.Mode.OUTPUT);
        channel.setState(false);
    }

    /**
     * update the LED status
     */
    public void draw () {

        switch (this.status) {
            case LED_BLINK: {
                channel.setState(isBlinkOn);
                long timeStamp = System.currentTimeMillis();
                if (timeStamp - blinkStartTime > BLINK_DURATION) {
                    blinkStartTime = timeStamp;
                    isBlinkOn = !isBlinkOn;
                }
                break;
            }
            case LED_ON:
                channel.setState(true);
                break;
            case LED_OFF:
            default:
                channel.setState(false);
                break;
        }
    }

    /**
     * set LED on
     */
    public void on() {
        this.status = LED_ON;
    }

    /**
     * set LED off
     */
    public void off() {
        this.status = LED_OFF;
    }

    /**
     * set LED blink
     */
    public void blink() {
        this.status = LED_BLINK;
    }

    /**
     * set LED on
     * @param on if LED is on
     */
    public void on(boolean on) {
        set(on);
    }

    /**
     * set LED on
     * @param on if LED is on
     */
    public void set(boolean on) {
        this.status = on ? LED_ON : LED_OFF;
    }

    /**
     * set LED status
     * @param status if LED is status
     */
    public void set(ELedStatus status) {
        this.status = status;
    }

    /**
     * get LED on
     * @return if LED is on
     */
    public boolean isOn () {
        return this.status == LED_ON;
    }

    /**
     * get LED status
     * @param status the given status
     * @return if LED is the given status
     */
    public boolean isStatus (ELedStatus status) {
        return this.status == status;
    }
}
