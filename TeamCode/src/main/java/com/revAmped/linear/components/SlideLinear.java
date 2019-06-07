package com.revAmped.linear.components;

import com.revAmped.components.HwMotor;
import com.revAmped.components.HwSwitch;
import com.revAmped.config.RobotConstants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Overcharged on 11/17/2017.
 * Class to manage the slides 
 * Move top, bottom & to encoder value
 */

public class SlideLinear
    extends HwMotorsLinear
{
	/** array of slides used
	*/
	
    public final HwMotor[] slides;

	/** Switch on the top to stop the slide up at the max
	*/
    public final HwSwitch switchSlideUp;
	/** Switch at the bottom to stop the slide down at
	*/
    public final HwSwitch switchSlideDown;

	/** Spacing used to position the slide at a preset value
	*/
    public final static int ENCODER_SPACING = 100;

	/**
	* Linear Op mode
	*/
    private LinearOpMode op;

	/** Construct the slide 
	* @param op a LinearOpMode type indicates the Op mode
	* @param switchSlideUp a HwSwitch object that is the top switch
    * @param switchSlideDown a HwSwitch object that is the bottom switch
    * @param slides a array of HwMotors that controls the slides movement
	*/
    public SlideLinear(LinearOpMode op,
                       HwSwitch switchSlideUp,
                       HwSwitch switchSlideDown,
                       HwMotor... slides) {
        super(op,
              slides);
        this.slides = slides;
        this.switchSlideUp = switchSlideUp;
        this.switchSlideDown = switchSlideDown;
        this.op = op;
    }

	/** Move slide to the bottom until it hits the bottom switch
	* Also reset the encoder value
	*/
    public void moveToBottom ()
            throws InterruptedException {
        while (op.opModeIsActive() &&
                !switchSlideDown.isTouch()) {
            for (HwMotor motor: slides) {
                motor.setPower(-RobotConstants.POWER_SLIDE);
            }
        }

        stop();
        resetPosition();
    }
	
	/** Move slide to the top until it hits the top switch
	*/
    public void moveToTop ()
            throws InterruptedException {
        while (op.opModeIsActive() &&
                !switchSlideUp.isTouch()) {
            for (HwMotor motor: slides) {
                motor.setPower(RobotConstants.POWER_SLIDE);
            }
        }
        stop();
    }

	/** Move slide to a preset position based on the encoder value
	* @param tick a int value to move the slide motors to the specific encoder value
	*/
    public void moveToEncoder (int tick)
            throws InterruptedException
    {
        float slidePower;
        while (op.opModeIsActive()) {
            int position = getCurrentPosition();

            if (tick > position + ENCODER_SPACING) {
                slidePower = RobotConstants.POWER_SLIDE;
            }
            else if (tick < position - ENCODER_SPACING) {
                slidePower = -RobotConstants.POWER_SLIDE;
            }
            else {
                break;
            }

            if ((slidePower < 0 && !switchSlideDown.isTouch()) ||
                    (slidePower > 0 && !switchSlideUp.isTouch())) {
                for (HwMotor motor: slides) {
                    motor.setPower(slidePower);
                }
                if (Math.abs(tick - position) < ENCODER_SPACING) {
                    break;
                }
            } else {
                break;
            }
        }

        stop();
        resetPosition();
    }

	/** Reset the slide encoder values to position 0
	*/
    public void resetPosition () {
        if (switchSlideDown.isTouch()) {
            for (HwMotor motor: slides) {
                motor.resetPosition(0);
            }
        }
    }

    /**
     *  get average encoder reading
     * @return average encoder reading
     */
    public int getCurrentPosition() {
        int sum = 0;
        for (HwMotor motor: slides) {
            sum += motor.getCurrentPosition();
        }
        return sum/slides.length;
    }

    /**
     * Set slide power
     * @param slidePower
     * @throws InterruptedException
     */
    public void setPower(float slidePower)
            throws InterruptedException {
        if ((slidePower < 0 && !switchSlideDown.isTouch()) ||
                (slidePower > 0 && !switchSlideUp.isTouch())) {
            for (HwMotor motor : slides) {
                motor.setPower(slidePower);
            }
        }
        else {
            stop();
            resetPosition();
        }
    }
}
