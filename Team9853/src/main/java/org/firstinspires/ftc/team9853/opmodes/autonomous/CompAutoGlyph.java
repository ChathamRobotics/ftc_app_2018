package org.firstinspires.ftc.team9853.opmodes.autonomous;

/*!
 * FTC_APP_2018
 * Copyright (c) 2017 Chatham Robotics
 * MIT License
 * @Last Modified by: storm
 * @Last Modified time: 2/2/2018
 */


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

import org.chathamrobotics.common.opmode.exceptions.StoppedException;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

public class CompAutoGlyph extends CompAuto {
    // MOTOR POSITIONS
    private static final int TICKS_PER_REV = 1120;
    private static final int POS_2_INTITIAL_POSITION = 1000;
    private static final int PLACE_GLYPH_POS = TICKS_PER_REV / 2;
    // {pos 1, pos 2}
    private static final int[] CLOSEST_COL_POSITIONS = {2 * TICKS_PER_REV, TICKS_PER_REV / 2};
    private static final int[] MIDDLE_COL_POSITIONS = {3 * TICKS_PER_REV, 3 * TICKS_PER_REV / 2};
    private static final int[] FARTHEST_COL_POSITIONS = {4 * TICKS_PER_REV, 5 * TICKS_PER_REV / 2};

    public CompAutoGlyph(boolean isRed, boolean isPos1) {
        super(isRed, isPos1);
    }

    @Override
    public void setup() {
        super.setup();

        robot.initVuforia();
    }

    @Override
    public void run() throws InterruptedException, StoppedException {
        // read vumark
        RelicRecoveryVuMark vuMark = robot.getVuMark();
        robot.log.info("VuMark", vuMark);

        // hit jewel
        super.run();

        ////// DRIVE TO CRYPTO BOX //////
        robot.driver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driver.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (! isPosition1) {
            robot.driver.setTargetPosition(POS_2_INTITIAL_POSITION);
            robot.driver.setDrivePower(90, AngleUnit.DEGREES, .75, 0);

            while (robot.driver.isBusy()) Thread.sleep(10);
        }

        robot.driver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driver.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int position;
        switch (vuMark) {
            case LEFT:
                position = (isRedTeam() ? FARTHEST_COL_POSITIONS : CLOSEST_COL_POSITIONS)[isPosition1 ? 0 : 1];
                break;
            case RIGHT:
                position = (isRedTeam() ? CLOSEST_COL_POSITIONS : FARTHEST_COL_POSITIONS)[isPosition1 ? 0 : 1];
                break;
            case CENTER:
            case UNKNOWN:
            default:
                position = MIDDLE_COL_POSITIONS[isPosition1 ? 0 : 1];
                break;
        }

        robot.driver.setTargetPosition(position);
        robot.driver.setDrivePower(isPosition1 ? 90 : (isRedTeam() ? 180 : 0), AngleUnit.DEGREES, .75, 0);

        while (robot.driver.isBusy()) Thread.sleep(10);

        // rotate
        if (isPosition1) {
            robot.driver.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.driver.setDrivePower(0, 0, 1);

            Thread.sleep(1000);
        }

        // place glyph
        robot.driver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driver.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.driver.setTargetPosition(PLACE_GLYPH_POS);
        robot.driver.setDrivePower(90, AngleUnit.DEGREES, .5, 0);

        while (robot.driver.isBusy()) Thread.sleep(10);

        robot.bottomGripper.open();

        robot.driver.setTargetPosition(0);
        robot.driver.setDrivePower(90, AngleUnit.DEGREES, .5, 0);

        while (robot.driver.isBusy()) Thread.sleep(10);

    }
}
