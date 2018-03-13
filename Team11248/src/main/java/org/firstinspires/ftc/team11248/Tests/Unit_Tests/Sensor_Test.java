package org.firstinspires.ftc.team11248.Tests.Unit_Tests;

/**
 * Created by tonytesoriero on 2/19/18.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team11248.RevRobot;

@TeleOp(name = "Sensor Test", group="Unit Test")
public class Sensor_Test extends OpMode{

    RevRobot robot;

    @Override
    public void init() {
        robot = new RevRobot(hardwareMap, telemetry);
        robot.safe_init();

        robot.setDriftMode(true);
        robot.jewelArm.setDriftMode(true);
        robot.backClaw.setDriftMode(true);
        robot.frontClaw.setDriftMode(true);
        robot.relicArm.setDriftMode(true);

        robot.vuforia.init(true,true);
    }

    @Override
    public void start(){
        robot.vuforia.activateTracking();
        robot.imu.setBaseline();
    }

    @Override
    public void loop() {
        robot.printSensorTelemetry();

        if(gamepad1.a){
            robot.resetEncoders();
            robot.jewelArm.resetCache();
            robot.jewelArm.setBaseLine();
            robot.vuforia.resetCache();
            robot.imu.setBaseline();
        }

    }

    @Override
    public void stop(){
        robot.imu.stop();
        robot.vuforia.deactivateTracking();
    }

}
