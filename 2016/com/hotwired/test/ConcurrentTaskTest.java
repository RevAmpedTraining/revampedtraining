package com.revAmped.test;

import com.revAmped.opmode.ConcurrentTask;
import com.revAmped.opmode.LinearTask;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by zwang on 8/29/2015.
 */
public class ConcurrentTaskTest extends LinearTask {
    MotorHelper mLeftDrive;
    // Members for tracking state information
    private ElapsedTime mStateTime = new ElapsedTime();
    private int mCount = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        double waitTime = 0;
        double speed = 0;
        mLeftDrive = new MotorHelper(hardwareMap.dcMotor.get("motor_1"), this);

        mLeftDrive.setMode(RunMode.RUN_USING_ENCODER);

        waitForStart();
        ConcurrentTask task = new MotorTestTask(this);

        mStateTime.reset();
        //sleep(20000);

        while (opModeIsActive()) {
            if (mStateTime.time() > waitTime) {
                mLeftDrive.setPower(speed);
                speed += 0.10;
                waitTime += 3;

                if (speed > 1.0) {
                    mCount++;
                    speed = 0;
                    mLeftDrive.resetEncoder();
                    mLeftDrive.setMode(RunMode.RUN_USING_ENCODER);
                }
            }
            telemetry.addData("EncoderL", Integer.toString(mLeftDrive.getCurrentPosition()));
            if (mCount > 3) {
                task.stop();
            }
            if (mCount > 4) {
                task.join();
            }

            idle();
        }
    }
}
