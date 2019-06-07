package com.revAmped.components;

import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

/**
 * Created by zwang on 2/14/2016.
 */
public class HwSwitch
    extends HwDevice
{
    private DeviceInterfaceModule dim;
    private int channel;
    private boolean isDigital;

    /**
     * initialize the switches
     * @param dim DIM to get switches from
     * @param id name of switches
     * @param channel channel to get switches from
     * @param isDigital digital port or not
     */
    public HwSwitch (DeviceInterfaceModule dim,
                     String id,
                     int channel,
                     boolean isDigital) {
        super(id);
        this.dim = dim;
        this.channel = channel;
        this.isDigital = isDigital;
    }

    /**
     * determines if the switch is pressed
     * @return status of the switch
     */
    public boolean isTouch () {
        return isDigital ? isTouchDigital(channel) : isTouchAnalog(channel);
    }

    // to do: use bitwise computation

    /**
     * digital switch logic
     * @param chan channel of switch
     * @return status of switch
     */
    private boolean isTouchDigital(int chan) {
        int logic = dim.getDigitalInputStateByte();
        switch (chan) {
            case 0:
                logic |= 0xFE;
                break;
            case 1:
                logic |= 0xFD;
                break;
            case 2:
                logic |= 0xFB;
                break;
            case 3:
                logic |= 0xF7;
                break;
            case 4:
                logic |= 0xEF;
                break;
            case 5:
                logic |= 0xDF;
                break;
            case 6:
                logic |= 0xBF;
                break;
            case 7:
                logic |= 0x7F;
                break;
            default:
                return false;
        }
        return logic == 255;
    }

    /**
     * analog switch logic
     * @param chan channel of switch
     * @return status of switch
     */
    private boolean isTouchAnalog(int chan) {
        double volt = dim.getAnalogInputVoltage(chan);
        return volt > 4.5;
    }
}
