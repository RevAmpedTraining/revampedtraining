package SimpleAuto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class JoannaSimpleAuto extends LinearOpMode {

    private DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get (DcMotor. class, "motor" );

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER );
        waitForStart();


    }
    public void driveForward(double power){
        motor.setPower (power);

    }
    public void stopDriving(){
        driveForward (0);

    }
    public void driveWithEncoder(double power, int distance ){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition (distance);
        driveForward(power);
        while(motor.isBusy()){

        }
        stopDriving();
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}
