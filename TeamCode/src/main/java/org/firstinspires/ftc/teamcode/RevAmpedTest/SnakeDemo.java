
package org.firstinspires.ftc.teamcode.RevAmpedTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import com.revAmped.components.SwerveDrive;
import com.revAmped.components.TurnType;
import com.revAmped.config.SwerveDriveConstants;
import com.revAmped.linear.components.RobotRevAmpedLinear;
import com.revAmped.linear.components.RobotRevAmpedLinearTest;
import com.revAmped.linear.components.SwerveDriveLinear;

/**
 * This OpMode ramps a single motor speed up and down repeatedly until Stop is pressed.
 * The code is structured as a LinearOpMode
 *
 * This code assumes a DC motor configured with the name "left_drive" as is found on a pushbot.
 *
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Snake Demo", group = "Test")
//@Disabled
public class SnakeDemo extends LinearOpMode {

    private RobotRevAmpedLinearTest robot;

    private SwerveDriveLinear drive;

    @Override
    public void runOpMode() {

        robot = new RobotRevAmpedLinearTest(this);
        drive = robot.getSwerveDriveLinear();

        waitForStart();

            for (int i=0; i<30; i++) {
                robot.servoRightBack.setPosition(SwerveDriveConstants.SERVO_RIGHTBACK_START -
                        2*i);
                robot.servoRightFront.setPosition(SwerveDriveConstants.SERVO_RIGHTFRONT_START +
                        2*i);
                robot.servoLeftBack.setPosition(SwerveDriveConstants.SERVO_LEFTBACK_START -
                        2*i);
                robot.servoLeftFront.setPosition(SwerveDriveConstants.SERVO_LEFTFRONT_START +
                        2*i);
                    drive.setPower(-0.5f,
                            -0.5f, TurnType.FORWARD);
                    idle();
                    sleep(50);
            }
            telemetry.addLine("Demo Done");
            telemetry.update();


    }

}