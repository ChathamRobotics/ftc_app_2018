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

    // MOTOR POSITIONS
    private static final int POS_2_INTITIAL_POSITION = 1000;
    // {pos 1, pos 2}
    private static final int[] CLOSEST_COL_POSITIONS = {5000, 100};
    private static final int[] MIDDLE_COL_POSITIONS = {4500, 300};
    private static final int[] FARTHEST_COL_POSITIONS = {3000, 400};

    private final boolean isPosition1, useLeftArm;

    public CompAuto(boolean isRed, boolean isPosition1) {
        super(isRed);

        this.isPosition1 = isPosition1;
        this.useLeftArm = ! isRedTeam();
    }

    @Override
    public void setup() {
        super.setup();

//        robot.initVuforia();

        reset();
    }

    @Override
    public void run() throws InterruptedException, StoppedException {
        RelicRecoveryVuMark vuMark;

        robot.start();
        robot.driver.setFront(RobotFace.BACK);

        /////// READ VUMARK ///////
//        vuMark = robot.getVuMark();
//        robot.log.info("VuMark", vuMark);

        /////// PICKUP GLYPH //////
        robot.bottomGripper.close();

        ////// HIT JEWEL //////
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

        ////// DRIVE TO CRYPTO BOX //////
//        robot.driver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.driver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        if (! isPosition1) {
//            robot.driver.setTargetPosition(POS_2_INTITIAL_POSITION);
//            robot.driver.setDrivePower(90, AngleUnit.DEGREES, .75, 0);
//
//            while (robot.driver.isBusy()) Thread.sleep(10);
//        }
//
//        int position;
//        switch (vuMark) {
//            case LEFT:
//                position = (isRedTeam() ? FARTHEST_COL_POSITIONS : CLOSEST_COL_POSITIONS)[isPosition1 ? 0 : 1];
//                break;
//            case RIGHT:
//                position = (isRedTeam() ? CLOSEST_COL_POSITIONS : FARTHEST_COL_POSITIONS)[isPosition1 ? 0 : 1];
//                break;
//            case CENTER:
//            case UNKNOWN:
//            default:
//                position = MIDDLE_COL_POSITIONS[isPosition1 ? 0 : 1];
//                break;
//        }
//
//        robot.driver.setTargetPosition(position);
//        robot.driver.setDrivePower(isPosition1 ? 90 : (isRedTeam() ? 180 : 0), AngleUnit.DEGREES, .75, 0);
//
//        while (robot.driver.isBusy()) Thread.sleep(10);
//
//        robot.driver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void reset() {
        robot.topGripper.close();
        robot.bottomGripper.close();

        robot.raiseRightArm();
        robot.raiseLeftArm();
    }
}
