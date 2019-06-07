package com.revAmped.components;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by zwang on 2/14/2016.
 */
public class HwSwitch
    extends HwDevice
{
    private AnalogInput analog;
    private DigitalChannel digital;
    private boolean isDigital;

    /**
     * initialize the switches
     * @param hardwareMap HardwareMap to get motor from
     * @param switchName name of switches
     * @param isDigital digital port or not
     */
    public HwSwitch (HardwareMap hardwareMap,
                     String switchName,
                     boolean isDigital) {
        super(switchName);
        this.isDigital = isDigital;
        if (isDigital) {
            // get a reference to our digitalTouch object.
            digital = hardwareMap.get(DigitalChannel.class, switchName);

            // set the digital channel to input.
            digital.setMode(DigitalChannel.Mode.INPUT);
        }
        else {
            analog = hardwareMap.get(AnalogInput.class, switchName);
        }
    }

    /**
     * determines if the switch is pressed
     * @return status of the switch
     */
    public boolean isTouch () {
        if (isDigital) {
            return digital.getState();
        }
        else {
            return analog.getVoltage() / analog.getMaxVoltage() > 0.99;
        }
    }
}
