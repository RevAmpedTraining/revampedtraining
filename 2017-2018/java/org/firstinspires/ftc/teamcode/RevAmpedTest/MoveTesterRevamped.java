package org.firstinspires.ftc.teamcode.LastYear.RevAmpedTest;

import com.qualcomm.robotcore.util.RobotLog;
import com.revAmped.components.Button;
import com.revAmped.components.TurnType;
import com.revAmped.config.RobotRevAmpedConstants;
import com.revAmped.linear.components.RobotRevAmpedLinear2;
import com.revAmped.linear.components.MecanumDriveLinear;
import com.revAmped.linear.util.SelectLinear;
import com.revAmped.linear.util.WaitLinear;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Move tester
 */
//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="MoveTesterReVamped2", group="Game")
public class MoveTesterRevamped
    extends LinearOpMode {
    private RobotRevAmpedLinear2 robot = null;

    private MecanumDriveLinear drive;
    private static final Button BTN_ROLLER_OUT = new Button();
    private enum Test {
        NONE,
        ALIGN,
        SONAR_RED_FAR,
        SONAR_BLUE_FAR,
        STRAFE_OFF_BOARD,
        DRIVE_AND_PIVOT_RED,
        DRIVE_AND_PIVOT_BLUE,
        DUMP;

        private static int numberTests = 0;

        public static Test getTest(int ordinal)
        {
            for (Test e : values()) {
                if (e.ordinal() == ordinal) {
                    return e;
                }
            }

            return NONE;
        }

        public static int getNumberTests() {
            if (numberTests == 0) {
                for (Test e : values()) {
                    numberTests++;
                }
            }
            return numberTests;
        }
    }

    /**
     * autonomous opMode
     */
    @Override
    public void runOpMode()
            throws InterruptedException {
        robot = new RobotRevAmpedLinear2(this);
        drive = robot.getMecanumDriveLinear();
        int testCounter = 0;
        Test currentTest = Test.NONE;

        //telemetry.addData("Waiting", "LinearTester");
        //telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            long timeStamp = System.currentTimeMillis();

            //choosing
            if(gamepad1.right_trigger > 0.9 && Button.BTN_NEXT.canPress(timeStamp)) {
                testCounter++;
                if(testCounter >= Test.getNumberTests()){
                    testCounter = 0;
                }
                currentTest = Test.getTest(testCounter);
            } else if(gamepad1.left_trigger > 0.9 && Button.BTN_PREV.canPress(timeStamp)) {
                testCounter--;
                if(testCounter < 0){
                    testCounter = Test.getNumberTests() - 1;
                }
                currentTest = Test.getTest(testCounter);
            }

            telemetry.addData("Test", currentTest);
            telemetry.addData("Select", "Next:RightTrigger Prev:LeftTrigger");
            telemetry.addData("Confirm", "Start");

            // test loop
            if (gamepad1.start && Button.BTN_START.canPress(timeStamp)) {
                switch(currentTest) {
                    case ALIGN:
                        alignAndDumpTest();
                        break;
                    case SONAR_RED_FAR:
                        sonarRedTest();
                        break;
                    case SONAR_BLUE_FAR:
                        sonarBlueTest();
                        break;
                    case STRAFE_OFF_BOARD:
                        strafeTest();
                        break;
                    case DRIVE_AND_PIVOT_BLUE:
                        driveOffNearTestB();
                        break;
                    case DRIVE_AND_PIVOT_RED:
                        driveOffNearTestR();
                        break;
                    case DUMP:
                        dumpTest();
                        break;
                    case NONE:
                    default:
                        break;
                }
            }
            telemetry.update();
            idle();
        }
    }

        WaitLinear lp = new WaitLinear(this);
        private void alignAndDumpTest()
                throws InterruptedException {
        robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_OUT_2);
        lp.waitMillis(1000);
        /*double distanceBack = robot.rangeStick.getDistance(DistanceUnit.CM);
        while (distanceBack>20) {
            drive.setPower(-0.32f);
            distanceBack = robot.rangeStick.getDistance(DistanceUnit.CM);
        }*/
        lp.waitMillis(50);
            long timestamp = System.currentTimeMillis();
            long endTime = timestamp;
        double alignDistance = robot.rangeStick.getDistance(DistanceUnit.CM);
        while (alignDistance < 6.25 || alignDistance > 6.75 && endTime < (timestamp+2000)) {
            if (alignDistance > 6.75) {
                drive.setStrafePower(0.33f);
                lp.waitMillis(160);
            } else {
                drive.setStrafePower(-0.33f);
                lp.waitMillis(160);
            }
            drive.stop();
            alignDistance = robot.rangeStick.getDistance(DistanceUnit.CM);
            endTime = System.currentTimeMillis();
        }
        //drive.moveToTime(0.45f, 200);
        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
        lp.waitMillis(400);
        drive.moveToTime(-0.5f, 400);
        drive.moveToTime(0.4f, 400);
        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
        robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_UP);
    }
    private void sonarRedTest()
        throws InterruptedException {
            int centerD = 55;
            int leftD = 68;
            int rightD = 42;
        double sonarWallBlue = robot.sonarR.getDistance();
        while (sonarWallBlue>rightD) {
            drive.setPower(0.6f, -0.6f, -0.6f, 0.6f);
            telemetry.addData("distance from wall", sonarWallBlue);
            sonarWallBlue = robot.sonarR.getDistance();
        }
        drive.turn(18,
                0.55f,
                5000,
                false);
        //drive.setTurnPower(0.4f);
        //lp.waitMillis(435);}
    }
    private void sonarBlueTest()
        throws  InterruptedException {
        double centerD = 17.5;
        double leftD = 14;
        int rightD = 21;
        double sonarWallBlue = robot.sonarR.getDistance();
        while (sonarWallBlue>rightD) {
            drive.setPower(0.6f, -0.6f, -0.6f, 0.6f);
            telemetry.addData("distance from wall", sonarWallBlue);
            sonarWallBlue = robot.sonarR.getDistance();
        }
        drive.turn(20,
                0.55f,
                4000,
                false);
        //drive.setTurnPower(0.4f);
        //lp.waitMillis(435);}
    }
    private void strafeTest() throws InterruptedException {
        drive.setPower(0.56f, -0.5f, -0.4f, 0.6f);
        lp.waitMillis(1420);
        /*drive.moveToEncoderInch(TurnType.STRAFE,
                38,
                0.8f,
                3000,
                true,
                true);*/
        drive.stop();
    }
    private void driveOffNearTestB()
        throws  InterruptedException {
        drive.moveToEncoderInch(TurnType.FORWARD,
                27,
                0.8f,
                3000,
                true,
                false);
        float headingStablize1 = robot.gyroSensor.getHeading();
        while (headingStablize1 > -75) {
            drive.setPower(0.4f, 0.4f, -0.4f, 0);
            headingStablize1 = robot.gyroSensor.getHeading();
            RobotLog.aa("heading turn 1", "%f", headingStablize1);
        }
        drive.turn(8, 0.6f, 4000,
                false);
    }
    private void driveOffNearTestR()
            throws  InterruptedException {
        drive.moveToEncoderInch(TurnType.FORWARD,
                27,
                0.8f,
                3000,
                true,
                false);
        float headingStablize1 = robot.gyroSensor.getHeading();
        while (headingStablize1 > -68) {
            drive.setPower(0.4f, 0.4f, -0.4f, 0);
            headingStablize1 = robot.gyroSensor.getHeading();
            RobotLog.aa("heading turn 1", "%f", headingStablize1);
        }
        drive.turn(15, 0.6f, 4000,
                false);
    }
    private void dumpTest()
            throws  InterruptedException {
        robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_IN);
        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_UP);
        drive.setTurnPower(-0.4f);
        lp.waitMillis(270);
        drive.moveToTime(-0.5f, 600);
        robot.servoDumperClaw.setPosition(RobotRevAmpedConstants.SERVO_DUMPER_CLAW_OUT);
        drive.moveToTime(0.5f, 300);
        robot.servoContainer.setPosition(RobotRevAmpedConstants.SERVO_CONTAINER_DOWN);
        robot.servoStick.setPosition(RobotRevAmpedConstants.SERVO_STICK_IN);
    }


}