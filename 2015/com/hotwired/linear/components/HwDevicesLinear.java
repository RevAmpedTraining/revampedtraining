package com.revAmped.linear.components;

import com.revAmped.components.HwDevice;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;

/**
 * This class perform an update multiple times until it is comfirmed.
 */
public class HwDevicesLinear
{
    protected HwDevice[] devices;

    protected LinearOpMode op;

    protected HwDevicesLinear(LinearOpMode op,
                              HwDevice... devices)
    {
        this.op = op;
        this.devices = devices;
    }

    protected HwDevicesLinear(HwDevice... devices)
    {
        this.devices = devices;
    }

    public void setLinearOpMode (LinearOpMode op) {
        this.op = op;
    }

    /**
     * This is used for update over multiple iterations. Sometimes
     * the update need to be done multiple times until
     * it is confirmed.
     * @param update update callback
     * @throws InterruptedException
     */
    protected void set (Update update)
        throws InterruptedException
    {
        if (devices == null || devices.length == 0) {
            return;
        }

        // init
        boolean[] isSets = new boolean[devices.length];
        Arrays.fill(isSets, false);

        boolean allSet;
        do {
            allSet = false;
            for (int i = 0; i < devices.length; i++) {
                if (!isSets[i]) {
                    if (update.isUpdated(devices[i])) {
                        isSets[i] = true;
                    } else {
                        update.update(devices[i]);
                    }
                }
                allSet |= isSets[i];
            }

            op.idle();
        }
        while (!allSet && op.opModeIsActive());
    }

    public interface Update {
        boolean isUpdated(HwDevice device);
        void update(HwDevice device);
    }
}
