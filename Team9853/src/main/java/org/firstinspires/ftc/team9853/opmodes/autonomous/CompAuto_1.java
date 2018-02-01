package org.firstinspires.ftc.team9853.opmodes.autonomous;

/*!
 * FTC_APP_2018
 * Copyright (c) 2017 Chatham Robotics
 * MIT License
 * @Last Modified by: storm
 * @Last Modified time: 1/30/2018
 */

import org.chathamrobotics.common.opmode.AutonomousRnB;
import org.chathamrobotics.common.opmode.exceptions.StoppedException;
import org.chathamrobotics.common.robot.RobotFace;
import org.firstinspires.ftc.team9853.opmodes.Autonomous9853;

@AutonomousRnB
public class CompAuto_1 extends Autonomous9853 {

    private final boolean useLeftArm;

    public CompAuto_1(boolean isRed) {
        super(isRed);

        useLeftArm = ! isRedTeam();
    }

    @Override
    public void setup() {
        super.setup();
        robot.topGripper.close();
        robot.bottomGripper.close();
    }

    @Override
    public void run() throws InterruptedException, StoppedException {
        robot.start();
        robot.driver.setFront(RobotFace.BACK);

        if (useLeftArm) robot.dropLeftArm();
        else robot.dropRightArm();

        debug();

        Thread.sleep(1000);
        int color = useLeftArm ? robot.getLeftColor() : robot.getRightColor();
        int rot = 0;
        if (isRedTeam() && robot.isRed(color) || (! isRedTeam() && robot.isBlue(color))) rot = -1;
        else if ((isRedTeam() && robot.isBlue(color)) || (! isRedTeam() && robot.isRed(color))) rot = 1;

        robot.driver.setDrivePower(0, 0, rot);
        debug();

        Thread.sleep(1000/4);
        robot.driver.stop();

        if (useLeftArm) robot.raiseLeftArm();
        else robot.raiseRightArm();

        robot.driver.setDrivePower(0, 0, -rot);
        debug();

        Thread.sleep(1000/4);
        robot.driver.stop();
    }
}
