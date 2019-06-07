package com.revAmped.test;

import com.revAmped.opmode.ConcurrentTask;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by zwang on 8/30/2015.
 */
public class MotorTestTask extends ConcurrentTask {
    MotorTestTask(LinearOpMode opMode) {
        super(opMode);
    }

    MotorHelper mRightDrive;
    // Members for tracking state information
    private ElapsedTime mStateTime2 = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        double waitTime2 = 0;
        double speed2 = 1.0;
        mRightDrive = new MotorHelper(getLinearOpMode().hardwareMap.dcMotor.get("motor_2"), this);

        mRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        mStateTime2.reset();

        while (opModeIsActive()) {
            if (mStateTime2.time() > waitTime2) {
                mRightDrive.setPower(speed2);
                speed2 -= 0.10;
                waitTime2 += 3;

                if (speed2 < 0) {
                    speed2 = 1.0;
                    mRightDrive.resetEncoder();
                    mRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }
            getLinearOpMode().telemetry.addData("EncoderR", Integer.toString(mRightDrive.getCurrentPosition()));

            idle();
        }
        getLinearOpMode().telemetry.addData("Concurrent", "End");
        mRightDrive.setPower(0);
    }
}
