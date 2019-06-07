package com.revAmped.test;

import com.revAmped.components.Button;
import com.revAmped.sensors.MaxSonarI2CXLRanger;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;

public class SonarTest
    extends LinearOpMode {

    private int addr = 224;

    private static final int ADDR_TICK = 2;
    private static final int MIN_ADDR = 2;
    private static final int MAX_ADDR = 254;

    @Override
    public void runOpMode() throws InterruptedException {


        MaxSonarI2CXLRanger sonar = new MaxSonarI2CXLRanger(hardwareMap.i2cDevice.get("i2cdev"));
        addr = searchAddr(sonar);
        try {
            //telemetry.setSorted(false);

            waitForStart();

            while (opModeIsActive()) {
                long timeStamp = System.currentTimeMillis();

                sonar.startRanging();
                Thread.sleep(80);
                int reading=sonar.getDistance();
                /*
                 * Send whatever telemetry data you want back to driver station.
                 */
                telemetry.addData("Distance", Integer.toString(reading));
                telemetry.addData("Change Address", "Start");
                Thread.sleep(20);

                if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {
                    int newAddr = selectAddr();
                    sonar.changeAddress(new I2cAddr(newAddr));
                    addr = newAddr;
                    //Wait 125ms for the sensor to save the new address and reset
                    Thread.sleep(200);
                }
                telemetry.addData("Address", String.format("0x%h", addr));
            }
        } finally {
            sonar.close();
        }
    }

    private int selectAddr () {
        int newAddr = addr;
        for (;;)
        {
            long timeStamp = System.currentTimeMillis();

            if (gamepad1.x && Button.BTN_MINUS.canPress4Short(timeStamp)) {
                newAddr -= ADDR_TICK;
                if (newAddr < MIN_ADDR) {
                    newAddr = MAX_ADDR;
                }
            }
            else if (gamepad1.b && Button.BTN_PLUS.canPress4Short(timeStamp)) {
                newAddr += ADDR_TICK;
                if (newAddr > MAX_ADDR) {
                    newAddr = MIN_ADDR;
                }
            }
            else if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {
                telemetry.addData("Address", String.format("0x%h", newAddr));
                break;
            }

            telemetry.addData("Adjust", "+:B -:X");
            telemetry.addData("Address", String.format("0x%h", newAddr));
            telemetry.addData("Select", "Start");
        }
        return newAddr;
    }

    private int searchAddr (MaxSonarI2CXLRanger sonar)
        throws InterruptedException {
        for (byte i = 2; i != 0; i += 2) {
            sonar.setI2cAddr(new I2cAddr(i));
            Thread.sleep(25);
            sonar.startRanging();
            Thread.sleep(100);
            int reading=sonar.getDistance();
            int addr = i > 0 ? i : 256 + i;

            if (reading >= 20) {
                telemetry.addData("Address", String.format("Found 0x%h", addr));
                return addr;
            }
            else {
                telemetry.addData("Address", String.format("Searching... 0x%h", addr));
            }
        }

        // never happen
        return 0;
    }
}
