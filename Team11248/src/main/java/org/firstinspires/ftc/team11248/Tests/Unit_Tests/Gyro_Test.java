package org.firstinspires.ftc.team11248.Tests.Unit_Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team11248.RevRobot;


@TeleOp(name = "MoveToAngle Test", group="Unit Test")
public class Gyro_Test extends OpMode {

    RevRobot robot;

    @Override
    public void init() {

        robot = new RevRobot(hardwareMap, telemetry);
        robot.safe_init();

    }

    @Override
    public void start() {
        robot.imu.setBaseline();
    }

    @Override
    public void loop() {
        robot.imu.logHeadingChanges();
        robot.moveToAngle(33.5);
        robot.printTelemetry();

    }

    @Override
    public void stop() {
        robot.imu.stop();
    }
}
