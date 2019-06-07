package com.revAmped.components;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by zwang on 2/18/2016.
 */
public class HwNavGyro
    extends HwGyro {

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    private NavxMicroNavigationSensor gyro;

    /**
     * initialize the gyro sensor
     * @param hardwareMap hardware map
     */
    public HwNavGyro(HardwareMap hardwareMap,
                     String gyroName)
        throws IllegalArgumentException, InterruptedException
    {
        super(gyroName);
        gyro = hardwareMap.get(NavxMicroNavigationSensor.class, gyroName);

        do {
            Thread.sleep(50);
        }
        while (gyro.isCalibrating());
    }

    /**
     * calibrate the gyro sensor
     */
    @Override
    public void calibrate() {}

    /**
     * get status of gyro sensor
     * @return if the gyro sensor is still calibrating
     */
    @Override
    public boolean isCalibrating() {
        return gyro.isCalibrating();
    }

    @Override
    public float getRawHeading() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;

    }
    @Override
    public float getRawRoll() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.secondAngle;
    }
    @Override
    public float getRawPitch() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.thirdAngle;
    }
}
