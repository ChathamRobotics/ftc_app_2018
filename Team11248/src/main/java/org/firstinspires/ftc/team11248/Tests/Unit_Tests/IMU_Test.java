package org.firstinspires.ftc.team11248.Tests.Unit_Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team11248.RevRobot;

/**
 * Created by tonytesoriero on 3/12/18.
 */


@TeleOp(name = "IMU Test", group="Unit Test")
public class IMU_Test extends OpMode {

    RevRobot robot;

    @Override
    public void init() {

        robot = new RevRobot(hardwareMap, telemetry);
        robot.imu.init();

    }

    @Override
    public void start(){
        robot.imu.setBaseline();
    }

    @Override
    public void loop() {

        robot.imu.logHeadingChanges();
        robot.imu.printTelemetry();
        telemetry.update();

    }

    @Override
    public void stop(){
        robot.imu.stop();
    }
}
