/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OldStuff;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.revAmped.config.RobotRevAmpedConstants;
        import com.revAmped.linear.components.RobotRevAmpedLinear;
        import com.revAmped.linear.components.TankDriveLinear;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.revAmped.linear.util.SelectLinear;
        import com.revAmped.linear.util.WaitLinear;
        import java.text.DecimalFormat;



/**
 *This is the autonomous program
 * 11/15/2017
 */
@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutonoRevamped", group="Game")

public class AutoRevamped extends LinearOpMode {


    private RobotRevAmpedLinear robot = null;
    private TankDriveLinear drive;
    private final static DecimalFormat numberFormatter = new DecimalFormat("######");
    int turn180 = 500;//encoder based turn. 500 ticks is approxtimate

    int slideup = 100;//moving the slide up once the glyph is clamped
    int currentPosition = robot.driveLeftFront.getCurrentPosition();




    @Override
    public void runOpMode()
            throws InterruptedException {
        try {
            //init
            robot = new RobotRevAmpedLinear(this);
            drive = robot.getTankDriveLinear();
            telemetry.addData("Calling Run", "Entering");
            run();
        }
        catch (Exception e) {
            e.printStackTrace();
        }
        finally {
            //shut down
            if (robot != null) {
                robot.close();
            }
        }
    }

    /**
     * autonomous function
     * @throws InterruptedException
     */

    // run until the end of the match (driver presses STOP)
    public void run()
            throws InterruptedException
    {
        SelectLinear sl = new SelectLinear(this);
        boolean isRed = sl.selectAlliance();
        telemetry.addData("Alliance", isRed ? "Red" : "Blue");

        //set the initial position.
        telemetry.update();




        WaitLinear lp = new WaitLinear(this);
        waitForStart();
        //grab the glyph at the start of auto
        robot.servoClawRight.setPosition(RobotRevAmpedConstants.SERVO_CLAW_RIGHT_IN);
        robot.servoClawLeft.setPosition(RobotRevAmpedConstants.SERVO_CLAW_LEFT_IN);
        //move the slide up so the glyph doesn't touch the ground
        currentPosition = robot.slideLeft.getCurrentPosition();
        while (currentPosition<slideup){
            robot.slideRight.setPower(0.5f);
            robot.slideLeft.setPower(0.5f);
            robot.slideLeft.getCurrentPosition();

        }
        //NOTE: When number of inches is positive the robot goes backwards. When negative it goes forwards
        //move to safe zone
        drive.moveToEncoderInch(
                -32,
                0.75f,
                10000,
                true,
                true,
                true);
        telemetry.addData("Front",
                "Left:" + numberFormatter.format(robot.driveLeftFront.getCurrentPosition()) +
                        " Right:" + numberFormatter.format(robot.driveRightFront.getCurrentPosition()));
        telemetry.addData("Back",
                "Left:" + numberFormatter.format(robot.driveLeftBack.getCurrentPosition()) +
                        " Right:" + numberFormatter.format(robot.driveRightBack.getCurrentPosition()));
        telemetry.update();
           /*drive.resetPosition();
           currentPosition = robot.driveLeftFront.getCurrentPosition();
           while (currentPosition<turn180) {
               robot.drive.setPower(.8f, -0.8f, TurnType.TURN_REGULAR);
               currentPosition = robot.driveLeftFront.getCurrentPosition();
           }
           drive.stop();
           drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/
        //NOTE: When number of inches is positive the robot goes backwards. When negative it goes forwards
        //move back and push in the glyph back in
        drive.moveToEncoderInch(
                10,
                0.75f,
                2000,
                false,
                true,
                true);
        //wait between backing out and lowering the slide back down
        lp.waitMillis(1000);
        while (!robot.switchSlideDown.isTouch()) {
            robot.slideLeft.setPower(0.5f);
            robot.slideRight.setPower(0.5f);
        }
        robot.servoClawRight.setPosition(RobotRevAmpedConstants.SERVO_CLAW_RIGHT_OUT);
        robot.servoClawLeft.setPosition(RobotRevAmpedConstants.SERVO_CLAW_LEFT_OUT);
        lp.waitMillis(1000);
        //theoretically push the glyph more into the box
        drive.moveToTime(0.6f, 3000);









        lp.waitMillis(2000);







    }

}

