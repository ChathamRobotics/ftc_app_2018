package org.firstinspires.ftc.team11248;

/**
 * Created by tonytesoriero on 2/19/18.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Sensor Test")
public class SensorTest extends OpMode{

    RevRobot robot;

    @Override
    public void init() {
        robot = new RevRobot(hardwareMap, telemetry);
        robot.init();

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
    }

    @Override
    public void loop() {
        telemetry.addData("Vuforia", "VuMark: " + robot.vuforia.getLastImage().toString());
        robot.printTelemetry();

        if(gamepad1.a){
            robot.resetEncoders();
            robot.jewelArm.resetCache();
            robot.vuforia.resetCache();
        }

    }
}
