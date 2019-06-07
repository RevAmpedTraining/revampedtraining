package com.revAmped.util;

import com.revAmped.linear.components.MoveAction;
import com.revAmped.linear.components.RobotRevAmpedLinear;
import com.revAmped.config.RobotRevAmpedConstants;

/**
 * Created by John Wang on 9/9/2018.
 */

public class MoveActionImpl implements MoveAction {

    private RobotRevAmpedLinear robot;

    public MoveActionImpl(RobotRevAmpedLinear robot) {
        this.robot = robot;
    }

    @Override
    public int distanceInTick() {
        return 0;
    }
    @Override
    public void perform() {

    }
    @Override
    public void stop() {

    }
}
