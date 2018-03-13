package org.firstinspires.ftc.team11248.Tests.Unit_Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team11248.Old_Files.Robot11248;
import org.firstinspires.ftc.team11248.Hardware.Vuforia_V2;
import org.firstinspires.ftc.team11248.RevRobot;

/**
 * Created by tonytesoriero on 9/11/17.
 */

@TeleOp( name = "VuForia Test", group="Unit Test")

public class Vuforia_Test extends OpMode {

    RevRobot robot;

    @Override
    public void init() {
        robot = new RevRobot(hardwareMap, telemetry);
        robot.vuforia.init(true,true);
    }

    @Override
    public void start(){
        robot.vuforia.activateTracking();
    }

    @Override
    public void loop() {
        robot.vuforia.printTelemetry();
        telemetry.update();

    }

    @Override
    public void stop(){
        robot.vuforia.deactivateTracking();
    }
}
