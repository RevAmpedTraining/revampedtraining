package com.revAmped.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MotorTimerTest extends LinearOpMode {

  long initTimestamp;
  DcMotor motor1, motor2;


    public void initialize() throws InterruptedException {
        hardwareMap.logDevices();


        try {
            motor1 = hardwareMap.dcMotor.get("motor1");
            motor1.setDirection(DcMotor.Direction.FORWARD);

        } catch (Exception e) {

            motor1 = null;
        }
        try {
            motor2 = hardwareMap.dcMotor.get("motor2");
            motor2.setDirection(DcMotor.Direction.FORWARD);

        } catch (Exception e) {

            motor2 = null;
        }

        initTimestamp = System.currentTimeMillis();
        telemetry.addData("Autonomous", " MotorTimerTest");
    }

    @Override
  public void runOpMode() throws InterruptedException {

    initialize();

    waitForStart();

    sleep(5000);


    if (motor1!=null) motor1.setPower(-0.75); //positive is out, negative is in

    delay(5000);

    if (motor1!=null) motor1.setPower(0);

    for (int i = 0; i < 100; i++) {
       idle();
    }
  }

  public void delay(long sleepTime)
  {
        long wakeupTime = System.currentTimeMillis() + sleepTime;

        while (sleepTime > 0)
        {
            try
            {
                Thread.sleep(sleepTime);
            }
            catch (InterruptedException e)
            {
            }
            sleepTime = wakeupTime - System.currentTimeMillis();
        }
  }//sleep

}
