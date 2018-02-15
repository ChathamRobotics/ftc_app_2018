package org.firstinspires.ftc.team11248.States_Robot.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team11248.States_Robot.RevRobot;

@TeleOp(name = "TEST DRIVE")
public class testdriv extends OpMode {

    RevRobot robot;

    @Override
    public void init() {

        robot = new RevRobot(hardwareMap, telemetry);

    }

    @Override
    public void loop() {

        //DRIVE TRAIN
        robot.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, true);
        robot.setFastMode(gamepad1.right_bumper);


        if (gamepad1.right_trigger > 0)
            robot.jewel.setPower(gamepad1.right_trigger);
        else if (gamepad1.left_trigger > 0)
            robot.jewel.setPower(-gamepad1.left_trigger);
        else
            robot.jewel.setPower(0);


        if (gamepad2.right_trigger > 0)
            robot.relicArm.setPower(gamepad2.right_trigger);
        else if (gamepad2.left_trigger > 0)
            robot.relicArm.setPower(-gamepad2.left_trigger);
        else
            robot.relicArm.setPower(0);

        robot.frontClaw.setPower(gamepad2.right_stick_y);
        robot.backClaw.setPower(gamepad2.left_stick_y);










    }
}

