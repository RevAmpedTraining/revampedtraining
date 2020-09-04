package org.firstinspires.ftc.teamcode.SquareAuto;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class JoannaAutoSquare extends OpMode {

    // defining the motor variables
    public DcMotor driveRF;
    public DcMotor driveRB;
    public DcMotor driveLF;
    public DcMotor driveLB;

    int loop = 0;


    // assigning the variables
    public void init (){
        driveRF = hardwareMap.get (DcMotor.class, "rf");
        driveRB = hardwareMap.get (DcMotor.class, "rb");
        driveLF = hardwareMap.get (DcMotor.class, "lf");
        driveLB = hardwareMap.get (DcMotor.class, "lb");
        driveLB.setDirection(DcMotor.Direction.REVERSE);
        driveRB.setDirection(DcMotor.Direction.REVERSE);
    }
    // this function makes the robot move forward
    public void DriveForward(double power) {
        driveLF.setPower(power);
        driveRF.setPower(power);
        driveLB.setPower(power);
        driveRB.setPower(power);
    }

    // this function makes the robot turn left
    public void DriveLeft(double power) {
        driveLF.setPower(-power * 0.5);
        driveRF.setPower(power * 0.5);
        driveLB.setPower(-power * 0.5);
        driveRB.setPower(power * 0.5);
    }

    //this function causes the robot to loop and make a square

    public void loop() {
        if (loop < 4) {
            DriveForward(1);
            //wait (1000);
            DriveLeft(1);
            //wait (1000);
            //adding a number so the loop does not go on infinite
            loop++;
        }
    }

    // this function stops and turns off the motors
    public void stop () {
        driveLF.close();
        driveRF.close();
        driveLB.close();
        driveRB.close();
    }
}
