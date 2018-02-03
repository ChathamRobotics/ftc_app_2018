package org.firstinspires.ftc.team9853.opmodes.autonomous;

/*!
 * FTC_APP_2018
 * Copyright (c) 2017 Chatham Robotics
 * MIT License
 * @Last Modified by: storm
 * @Last Modified time: 2/2/2018
 */


import org.chathamrobotics.common.opmode.exceptions.StoppedException;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class CompAutoPark extends CompAuto {
    public CompAutoPark(boolean isRed, boolean isPos1) {
        super(isRed, isPos1);
    }

    @Override
    public void run() throws InterruptedException, StoppedException {
        super.run();

        robot.driver.setDrivePower(90, AngleUnit.DEGREES, 0.5, 0);
        Thread.sleep(1000);
        robot.driver.setDrivePower(0,0);
    }
}
