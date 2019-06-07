
package org.firstinspires.ftc.teamcode.RevAmpedTest;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import com.revAmped.components.SwerveDrive;
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
@TeleOp(name = "Popper Reset", group = "Test")
//@Disabled
public class PopperResetTest extends LinearOpMode {

    private RobotRevAmpedLinearTest robot;

    private SwerveDriveLinear drive;

    static final double INCREMENT   = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV     = -1.0;     // Maximum REV power applied to motor

    // Define class members
    //DcMotor motor;
    double  power   = 0;
    boolean rampUp  = true;
    boolean ENCreset  = true;
    int newTarget;


    int count_per_turn = 1120; // this is Count per turn
    double degree_target_abs = 359; // this is Abs dress target  final position shouls be inside 300-359.99
    double count_current = 0; // this is Abs degree target + N* turn
    double count_to_go = 0; //
    double temp = 3%1;


    @Override
    public void runOpMode() {

        robot = new RobotRevAmpedLinearTest(this);
        drive = robot.getSwerveDriveLinear();
        // Connect to motor (Assume standard left wheel)
        // Change the text in quotes to match any motor name on your robot.
        //motor = hardwareMap.get(DcMotor.class, "left_drive");

        if (ENCreset) { // only rest once

            // one way to reset 0 deg is mannuall at begining

            // 2nd way roll back to cliff, it will sop

            robot.motorPopper.setPower(-0.15f);  // -0.o8 not move
            sleep(2000);  // need run >2 sec for worst case to gget to the point


            robot.motorPopper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ENCreset = !ENCreset;
        }

        // Wait for the start button
        telemetry.addData(">", "Motro rested and Press Start to run Motors." );
        telemetry.update();




        waitForStart();

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive()) {


            telemetry.update();


            ///////////////

            //   https://ftc-tricks.com/dc-motors/
            // and PushbotAutoDriveByEncoder_Linear
            // Set the motor to the new power and pause;
            //motor.setPower(power);
            // this hsould be moved to INIT at beginne

            if (gamepad2.a) {
                robot.motorPopper.setPower(1f);
            } else {

                robot.motorPopper.setPower(0);  // this seems importnat, other wise, it could trapped into while wait loop

                //sleep(250);  // may need dealy to measure
                count_current = robot.motorPopper.getCurrentPosition()  % count_per_turn; // convert to in one cycle ie 370->10
                //count_to_go = (count_per_turn) - count_current;
                count_to_go=count_current;
                telemetry.addData("current", "Starting at %7f ", count_current);
                telemetry.addData("count2go", "Starting at %7f ", count_to_go);
                if(count_to_go > 0) {  // if not in sweet point....

                    newTarget = robot.motorPopper.getCurrentPosition() - (int) (count_to_go) + 30 ; // add 30 count ~ 1 degree, make sure 2nd time alway pass.....
                    telemetry.addData("target GO ", "Starting at %7d ", newTarget);


                    //mothod 1 : use RUN TO POSITION MODE
                    //  degree_target_current = 200%count_per_turn
                    //motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.motorPopper.setTargetPosition(newTarget);
                    // Turn On RUN_TO_POSITION
                    robot.motorPopper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    // reset the timeout time and start motion.
                    robot.motorPopper.setPower(-0.75f); // if this number is too big could oscillatiion
                    //motor.setPower(0);

                    while (opModeIsActive() &&
                            (robot.motorPopper.isBusy() )) {
                        // https://ftcforum.usfirst.org/forum/ftc-technology/android-studio/7148-encoder-trouble

                        // Display it for the driver.
                        telemetry.addData("current", "begining  ", count_current);
                        telemetry.addData("count2go", "begining Starting at %7f ", count_to_go);
                        telemetry.addData("Path2",  "current target cuurentmod at %7d :%7d:%7d",
                                robot.motorPopper.getCurrentPosition(),
                                newTarget,
                                robot.motorPopper.getCurrentPosition()% count_per_turn);
                        telemetry.update();
                    }
                    /*robot.motorPopper.setPower(0.75f);
                    try {
                        Thread.sleep(250);
                    }catch(Exception e) {}

                    robot.motorPopper.setPower(0);*/

                }
                else {
                    robot.motorPopper.setPower(0);
                    telemetry.addData(">", "lucky no need run." );
                    // telemetry.addData("target meet,NO need GO ");

                }
                robot.motorPopper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   // back to normal mode
                robot.motorPopper.setPower(0);
            }  // if not push a
        }


        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path1",  "Starting at %7f ",
                count_current
        );


        sleep(CYCLE_MS);
        idle();


        // Turn off motor and signal done;


        robot.motorPopper.setPower(0);
        telemetry.addData(">", "Done");
        telemetry.update();

    }

}