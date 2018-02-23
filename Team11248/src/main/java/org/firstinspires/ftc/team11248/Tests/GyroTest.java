package org.firstinspires.ftc.team11248.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team11248.RevRobot;


@TeleOp(name = "GyroTest")
public class GyroTest extends OpMode {

    RevRobot robot;

    @Override
    public void init() {

        robot = new RevRobot(hardwareMap, telemetry);
        robot.init();
        robot.init_IMU();

    }

    @Override
    public void loop() {

      robot.moveToAngle(33.5);

    }
}
