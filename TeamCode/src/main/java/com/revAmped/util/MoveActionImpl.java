package com.revAmped.util;

import com.revAmped.linear.components.MoveAction;
import com.revAmped.linear.components.RobotRevAmpedLinear;
import com.revAmped.config.RobotRevAmpedConstants;
import com.revAmped.linear.components.RobotRevAmpedLinearTest;

/**
 * Created by John Wang on 9/9/2018.
 */

public class MoveActionImpl implements MoveAction  {

    private RobotRevAmpedLinearTest robot;

    public MoveActionImpl(RobotRevAmpedLinearTest robot) {
        this.robot = robot;
    }

    @Override
    public int distanceInTick() {
        return 0;
    }
    @Override
    public void perform() {
        boolean isSlideDown = robot.switchSlideDown.isTouch();
        if (!isSlideDown) {
            robot.motorLatch.setPower(-1);
        }
    }
    @Override
    public void stop() {

    }
}
