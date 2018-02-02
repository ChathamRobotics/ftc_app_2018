package org.firstinspires.ftc.team9853.opmodes.teleop;

/*!
 * FTC_APP_2018
 * Copyright (c) 2017 Chatham Robotics
 * MIT License
 * @Last Modified by: storm
 * @Last Modified time: 11/26/2017
 */

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.chathamrobotics.common.Controller;
import org.chathamrobotics.common.robot.RobotFace;
import org.chathamrobotics.common.systems.Gripper;
import org.firstinspires.ftc.team9853.opmodes.Tele9853;

/**
 *
 */
@SuppressWarnings("unused")
@TeleOp(name = "CompDrive")
public class CompDrive extends Tele9853 {
    @Override
    public void loop() {
        if (controller1.right_trigger > 0)
            robot.setLiftPower(controller1.right_trigger);
        else
            robot.setLiftPower(
                    -controller1.left_trigger
            );

        if (controller1.aState == Controller.ButtonState.TAPPED) {
            robot.topGripper.grip();
            robot.bottomGripper.grip();
        }

        if (controller1.xState == Controller.ButtonState.TAPPED) {
            robot.topGripper.open();
            robot.bottomGripper.open();
        }

        if (controller1.rightBumperState == Controller.ButtonState.TAPPED) {
            if (robot.bottomGripper.getState() == Gripper.State.OPEN) robot.bottomGripper.grip();
            else robot.bottomGripper.open();
        }

        if (controller1.leftBumperState == Controller.ButtonState.TAPPED) {
            if (robot.topGripper.getState() == Gripper.State.OPEN) robot.topGripper.grip();
            else robot.topGripper.open();
        }

        float x = controller1.left_stick_x, y = -controller1.left_stick_y, rotation = controller1.right_stick_x;
        double magnitude = Math.hypot(x, y);
        double direction = Math.atan2(y, x);

        if (controller1.padUpState == Controller.ButtonState.TAPPED)
            robot.driver.setFront(RobotFace.FRONT);
        if (controller1.padDownState == Controller.ButtonState.TAPPED)
            robot.driver.setFront(RobotFace.BACK);
        if (controller1.padLeftState == Controller.ButtonState.TAPPED)
            robot.driver.setFront(RobotFace.LEFT);
        if (controller1.padRightState == Controller.ButtonState.TAPPED)
            robot.driver.setFront(RobotFace.RIGHT);

        robot.driver.setDrivePower(direction, magnitude, rotation);
    }
}
