package com.revAmped.components;

import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

/**
 * Created by zwang on 2/16/2016.
 */
public class HwLed
    extends HwDevice {

    private DeviceInterfaceModule dim;
    private int channel;

    private boolean on = false;

    /**
     * initialize the LEDs
     * @param dim DIM to get LEDs from
     * @param channel channel to get LEDs from
     */
    public HwLed(DeviceInterfaceModule dim,
                 String id,
                 int channel) {
        super(id);
        this.dim = dim;
        this.channel = channel;

        dim.setDigitalChannelMode(channel, DigitalChannelController.Mode.OUTPUT);
    }

    /**
     * update the LED statuses
     */
    public void draw () {
        dim.setDigitalChannelState(channel, on);
    }

    /**
     * set LED on
     */
    public void on() {
        this.on = true;
    }

    /**
     * set LED off
     */
    public void off() {
        this.on = false;
    }

    /**
     * set LED status
     * @param on if LED is on
     */
    public void on(boolean on) {
        this.on = on;
    }

    /**
     * set LED status
     * @param on if LED is on
     */
    public void set(boolean on) {
        this.on = on;
    }
    /**
     * get LED status
     * @return if LED is on
     */
    public boolean isOn () {
        return on;
    }
}
