package com.revAmped.linear.util;

import com.revAmped.components.Button;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * For selection using game pad before waitForStart()
 */
public class SelectLinear
{
    private LinearOpMode op;

    public SelectLinear(LinearOpMode op) {
        this.op = op;
    }

    /**
     * Adjust delay in milliseconds
     * @return delay in milliseconds
     * @throws InterruptedException
     */
    public int adjustDelay ()
        throws InterruptedException {
        // 30 second max
        return adjust("Delay", 30) * 1000;
    }

    /**
     * Adjust a number
     * @return a number
     * @throws InterruptedException
     */
    public int adjust (String title,
                       int max)
        throws InterruptedException
    {
        op.gamepad1.reset();
        int number = 0;
        while (!Thread.currentThread().isInterrupted()) {
            long timeStamp = System.currentTimeMillis();

            //choosing
            if (op.gamepad1.b && Button.BTN_PLUS.canPressShort(timeStamp)) {
                number += 1;
                if (number > max) {
                    number = max;
                }
            } else if (op.gamepad1.x && Button.BTN_MINUS.canPressShort(timeStamp)) {
                number -= 1;
                if (number < 0) {
                    number = 0;
                }
            }
            else if (op.gamepad1.start && Button.BTN_START.canPress(timeStamp)) {
                break;
            }

            op.telemetry.addData(title + " adjust", "+:B  -:X");
            op.telemetry.addData("Confirm", "Start");
            op.telemetry.addData(title, number);
            op.telemetry.update();

            op.idle();
        }

        RobotLog.i("Adjust: " + title + "=" + number);
        op.idle();
        return number;
    }

    /**
     * Select alliance
     * @return true for red, false for blue
     * @throws InterruptedException
     */
    public boolean selectAlliance ()
        throws InterruptedException {

        String[] alliances = new String[] {"Red", "Blue"};
        int index = select(alliances,
                           "Alliance");
        return index == 0;
    }

    /**
     * Select position
     * @return true for near, false for far
     * @throws InterruptedException
     */
    public boolean selectPosition ()
            throws InterruptedException {

        String[] positions = new String[] {"Left", "Right"};
        int index = select(positions,
                "Position");
        return index == 0;
    }

    /**
     * Select extra glyphs in auto
     * @throws InterruptedException
     */
    public boolean extraGlyph ()
            throws InterruptedException {

        String[] glyph = new String[] {"Yes", "No"};
        int index = select(glyph,
                "four glyph");
        return index == 0;
    }
    /**
     * Select extra glyphs in auto
     * @throws InterruptedException
     */
    public boolean extraGlyph2 ()
            throws InterruptedException {

        String[] glyph = new String[] {"Yes", "No"};
        int index = select(glyph,
                "six glyph");
        return index == 0;
    }
    /**
     * Select extra glyphs in auto
     * @throws InterruptedException
     */
    public boolean jewel ()
            throws InterruptedException {

        String[] glyph = new String[] {"Yes", "No"};
        int index = select(glyph,
                "opencv");
        return index == 0;
    }
    /**
     * Select backoff distance
     * @return true for near, false for far
     * @throws InterruptedException
     */
    public boolean selectBackoff ()
            throws InterruptedException {

        String[] backoffNear = new String[] {"Near", "Far"};
        int index = select(backoffNear,
                "Backoff Distance");
        return index == 0;
    }

    /**
     * Select from the given array
     * @return index from the given array starting 0
     * @throws InterruptedException
     */
    public int select (Object[] list,
                       String title)
        throws InterruptedException {
        int index = 0;

        op.gamepad1.reset();
        while (!Thread.currentThread().isInterrupted()) {
            long timeStamp = System.currentTimeMillis();
            if(op.gamepad1.b && Button.BTN_NEXT.canPress(timeStamp)) {
                index++;
                if(index >= list.length){
                    index = 0;
                }
            } else if(op.gamepad1.x && Button.BTN_PREV.canPress(timeStamp)) {
                index--;
                if(index < 0){
                    index = list.length - 1;
                }
            }
            else if (op.gamepad1.start && Button.BTN_START.canPress(timeStamp)) {
                break;
            }

            op.telemetry.addData(title, list[index]);
            op.telemetry.addData("Select", "Next: B  Prev: X");
            op.telemetry.addData("Confirm", "Start");
            op.telemetry.update();

            op.idle();
        }

        RobotLog.i("Select: " + title + "=" + list[index]);
        op.idle();
        return index;
    }

    /**
     * Confirm selection
     * @param title item to be confirmed
     * @param def default value
     * @return true to confirm
     */
    public boolean confirm (String title,
                            boolean def)
        throws InterruptedException {

        String[] options;
        if (def) {
            options = new String[]{"Yes", "No"};
        } else {
            options = new String[]{"No", "Yes"};
        }

        int index = select(options,
                           title);
        return index == (def ? 0 : 1);
    }
}
