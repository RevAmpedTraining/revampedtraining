import com.qualcomm.robotcore.hardware.DcMotor;

public class LinearOpModeFirstAttempt extends LinearOpMode{

    private DcMotor

    @override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "motor");

        waitforstart();

        while (runOpModeIsActive())
            motor.setpower(0.5f);
    }

    }

}
