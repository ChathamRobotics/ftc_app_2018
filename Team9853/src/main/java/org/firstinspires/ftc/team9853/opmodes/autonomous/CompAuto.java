package org.firstinspires.ftc.team9853.opmodes.autonomous;

/*!
 * FTC_APP_2018
 * Copyright (c) 2017 Chatham Robotics
 * MIT License
 * @Last Modified by: storm
 * @Last Modified time: 1/30/2018
 */

import com.qualcomm.robotcore.hardware.DcMotor;

import org.chathamrobotics.common.opmode.exceptions.StoppedException;
import org.chathamrobotics.common.robot.RobotFace;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.team9853.opmodes.Autonomous9853;

public class CompAuto extends Autonomous9853 {
    public final boolean isPosition1, useLeftArm;

    public CompAuto(boolean isRed, boolean isPosition1) {
        super(isRed);

        this.isPosition1 = isPosition1;
        this.useLeftArm = ! isRedTeam();
    }

    @Override
    public void setup() {
        super.setup();

        reset();
    }

    @Override
    public void run() throws InterruptedException, StoppedException {

        robot.start();
        robot.driver.setFront(RobotFace.BACK);

        /////// PICKUP GLYPH //////
        robot.log.info("Grabbing glyph");
        robot.bottomGripper.grip();

        ////// HIT JEWEL //////
        robot.log.info("Dropping Arm");
        if (useLeftArm) robot.dropLeftArm();
        else robot.dropRightArm();

        debug();

        robot.log.info("Getting Color");
        Thread.sleep(1000);
        int color = 0, rot = 0;
        long endTime = System.currentTimeMillis() + 1000;

        while (color == 0 && System.currentTimeMillis() < endTime) {
            color = useLeftArm ? robot.getLeftColor() : robot.getRightColor();
            Thread.sleep(100);
        }

        robot.log.info("Found Color Number", color);
        if (isRedTeam() && robot.isRed(color) || (! isRedTeam() && robot.isBlue(color))) rot = 1;
        else if ((isRedTeam() && robot.isBlue(color)) || (! isRedTeam() && robot.isRed(color))) rot = -1;

        robot.log.info("Hitting Jewel");
        robot.driver.setDrivePower(0, 0, rot);
        debug();

        Thread.sleep(1000/4);
        robot.driver.stop();

        robot.log.info("Raising Arm");
        if (useLeftArm) robot.raiseLeftArm();
        else robot.raiseRightArm();

        robot.log.info("Repositioning");
        robot.driver.setDrivePower(0, 0, -rot);
        debug();

        Thread.sleep(1000/4);
        robot.driver.stop();
        telemetry.update();
    }

    private void reset() {
        robot.topGripper.close();
        robot.bottomGripper.close();

        robot.raiseRightArm();
        robot.raiseLeftArm();
    }
}
