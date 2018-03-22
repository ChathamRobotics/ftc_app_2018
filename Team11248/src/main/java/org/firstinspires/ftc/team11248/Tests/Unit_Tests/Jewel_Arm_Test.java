package org.firstinspires.ftc.team11248.Tests.Unit_Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.team11248.Hardware.Jewel_Arm;
import org.firstinspires.ftc.team11248.RevRobot;

/**
 * Created by tonytesoriero on 3/12/18.
 */


@TeleOp(name = "Jewel Arm Test", group="Unit Test")
public class Jewel_Arm_Test extends OpMode {

    private RevRobot robot;
    private double power = 0;

    private boolean lastJewelArmPress = false;

    @Override
    public void init() {
        robot = new RevRobot(hardwareMap, telemetry);
        robot.jewelArm.init();
    }

    @Override
    public void start(){
        robot.jewelArm.setBaseLine();
        robot.jewelArm.resetEncoders();
    }

    @Override
    public void loop() {

        if(gamepad1.a){
            robot.jewelArm.resetCache();
            robot.jewelArm.setBaseLine();
        }


        if(robot.jewelArm.pressed() && !lastJewelArmPress) {

            if(power == 0){
                power = robot.jewelArm.getCurrentPosition() > robot.jewelArm.MAX_ENCODER_COUNT?-.5:.5;

            }else {
                power = 0;
            }

        }

        if(robot.jewelArm.getCurrentPosition() > robot.jewelArm.MAX_ENCODER_COUNT || robot.jewelArm.getCurrentPosition() < 0) power = 0;

        lastJewelArmPress = robot.jewelArm.pressed();

        robot.jewelArm.setPower(power);

        robot.jewelArm.printTelemetry();
        telemetry.update();


    }

    @Override
    public void stop(){


    }
}
