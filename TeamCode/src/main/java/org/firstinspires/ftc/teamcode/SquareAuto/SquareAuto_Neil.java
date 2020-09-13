package org.firstinspires.ftc.teamcode.SquareAuto;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SquareAuto_Neil extends OpMode {

    // Define the variables for each motor...
    public DcMotor driveRF;
    public DcMotor driveRB;
    public DcMotor driveLF;
    public DcMotor driveLB;

    // Assign the variables to the hardware...
    public void init (){
        driveRF = hardwareMap.get (DcMotor.class, "rf");
        driveRB = hardwareMap.get (DcMotor.class, "rb");
        driveLF = hardwareMap.get (DcMotor.class, "lf");
        driveLB = hardwareMap.get (DcMotor.class, "lb");
        driveLB.setDirection(DcMotor.Direction.REVERSE);
        driveRB.setDirection(DcMotor.Direction.REVERSE);

    }
    // Move the robot forward...
    public void DriveForward(double power) {
        driveLF.setPower(power);
        driveRF.setPower(power);
        driveLB.setPower(power);
        driveRB.setPower(power);

    }

    // Turn the robot left...
    public void DriveLeft(double power) {
        driveLF.setPower(-power*0.5);
        driveRF.setPower(power*0.5);
        driveLB.setPower(-power*0.5);
        driveRB.setPower(power*0.5);

    }

    //this function causes the robot to loop and make a square
    public void loop() {
        int loop = 1;
        while(loop <= 4) {
            DriveForward(1);

            DriveLeft(1);

            // Marking the loop's increase to get to 4 loops...
            loop = loop + 1;

        }

    }

    // Stop the program and motors...
    public void stop () {
        driveLF.close();
        driveRF.close();
        driveLB.close();
        driveRB.close();

    }
}
