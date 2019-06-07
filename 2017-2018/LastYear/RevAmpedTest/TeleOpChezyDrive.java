package org.firstinspires.ftc.teamcode.LastYear.RevAmpedTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
import com.revAmped.components.RobotRevAmped2;
import com.revAmped.components.MecanumDrive;

import java.text.DecimalFormat;

/**
 * Created by swang4 on 8/2/2018.
 */
//@Disabled
public class TeleOpChezyDrive  extends OpMode {

    private RobotRevAmped2 robot;

    private MecanumDrive drive;

    private float pwrDrive, pwrTurn;


    private final static DecimalFormat numberFormatter = new DecimalFormat("######");

    // private TipOverProtection forwordProtection = new TipOverProtection(true);
    //private TipOverProtection backwordProtection = new TipOverProtection(false);

    @Override
    public void init() {
        robot = new RobotRevAmped2(this,
                false);
        drive = robot.getMecanumDrive();

        this.gamepad1.reset();
        this.gamepad1.setJoystickDeadzone(0.15f);
        this.gamepad2.reset();
        this.gamepad2.setJoystickDeadzone(0.15f);
    }

    @Override
    public void stop() {
        robot.close();
    }

    @Override
    public void loop() {

        long timestamp = System.currentTimeMillis();

        // is reversed by default
        //float x1 = gamepad1.left_stick_x;
        //float y1 = -gamepad1.left_stick_y;
        //float x2 = gamepad1.right_stick_x;
        float y2 = -gamepad1.right_stick_y;

        float tx1 = gamepad2.left_stick_x;
        //float ty1 = gamepad2.left_stick_y;
        //float tx2 = gamepad2.right_stick_x;
        //float ty2 = gamepad2.right_stick_y;

        //pwrTurn = Button.scaleInput(tx1);
        //pwrDrive = Button.scaleInput(y2);

        pwrDrive = Range.clip(pwrDrive, -1f, 1f);
        pwrTurn = Range.clip(pwrTurn, -1f, 1f);

        if (pwrTurn < 0.15f && pwrTurn > -0.15f) {
            drive.setPower(pwrDrive, pwrDrive);
        } else if (pwrTurn < -0.15f) {
            drive.setPower(pwrDrive - pwrTurn * pwrDrive, pwrDrive + pwrTurn * pwrDrive);
        } else {
            drive.setPower(pwrDrive + pwrTurn * pwrDrive, pwrDrive - pwrTurn * pwrDrive);
        }
    }
}
