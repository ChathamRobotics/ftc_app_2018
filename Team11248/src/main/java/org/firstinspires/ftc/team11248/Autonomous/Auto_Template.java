package org.firstinspires.ftc.team11248.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team11248.Hardware.Claw;
import org.firstinspires.ftc.team11248.RevRobot;

/**
 * Created by tonytesoriero on 2/19/18.
 */

public class Auto_Template extends LinearOpMode{

    private boolean isBlueAlliance;

    private RevRobot robot;
    private Claw claw;

    private int state = 0;

    Auto_Template(boolean isBlueAlliance){
        this.isBlueAlliance = isBlueAlliance;

    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RevRobot(hardwareMap, telemetry);
        robot.init();

        claw = !isBlueAlliance ? robot.frontClaw : robot.backClaw;

        robot.vuforia.init(true,true);
        robot.vuforia.activateTracking();

        waitForStart();

        switch (state){

            case 0: //Grab Glyph and start first jewelArm move

                claw.grabTop();
                sleep(200);
                claw.setPower(1);
                sleep(375);
                claw.setPower(0);

                robot.jewelArm.setPower(1);
                sleep(1000);

                state++;
                break;



            case 1: //Extend jewelArm till hits wall or maxes out

                robot.jewelArm.setPower(.25);

                if(robot.jewelArm.pressed() ){
                    robot.jewelArm.setPower(0);
                    state++;
                }

                if (robot.jewelArm.getCurrentPosition() >= robot.jewelArm.MAX_ENCODER_COUNT){
                    robot.jewelArm.setPower(0);
                    state += 2;
                }

                break;


            case 2: //Back up to read jewel

                break;

            case 3: //Read jewel color

                break;

            case 4: //Retract jewelArm
                break;

            case 5: //
                break;

        }
    }
}
