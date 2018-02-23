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
        robot.init_IMU();

        claw = !isBlueAlliance ? robot.frontClaw : robot.backClaw; // with ! drags glyph (back claw)

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

                robot.jewelArm.setBaseLine();

                state++;
                break;



            case 1: //Extend jewelArm till hits wall or maxes out

                robot.jewelArm.setPower(.25);

                if(robot.jewelArm.pressed() ){
                    robot.jewelArm.stop();
                    robot.jewelArm.rotationsToWall = robot.jewelArm.getCurrentPosition();
                    state++;
                }

                if (robot.jewelArm.getCurrentPosition() >= robot.jewelArm.MAX_ENCODER_COUNT){
                    robot.jewelArm.stop();
                    state += 2;
                }
                break;


            case 2: //Back up to read jewel

                robot.jewelArm.setPower(-.25);
                int targetPosition = robot.jewelArm.rotationsToWall - robot.jewelArm.BACK_UP_ROTATIONS;

                if (robot.jewelArm.getCurrentPosition() <= targetPosition ){
                    robot.jewelArm.stop();
                    state++;
                }

                break;

            case 3: //Read jewel color


                //if sees one, chck if its lft right
                //if nothing go to state with moving the arm back in

                boolean isLeftJewelRed = robot.jewelArm.isRed();
                boolean isLeftJewelBlue = robot.jewelArm.isBlue();

                if( !(isLeftJewelBlue == isLeftJewelRed) ) {

                    robot.drive(0, .5 * ((isBlueAlliance?isLeftJewelRed:isLeftJewelBlue) ? 1 : -1), 0);
                    sleep(500);
                    robot.setDriftMode(true);

                    robot.stop();
                    robot.setDriftMode(false);


                    sleep(500);

                    if(isLeftJewelBlue){
                        robot.drive(0,-1 * (isBlueAlliance?1:-1), 0);
                        sleep(750);
                        robot.stop();

                    } else {
                        robot.drive(0, isBlueAlliance?1:-1, 0);
                        sleep(1500);

                        robot.setDriftMode(true);
                        sleep(500);
                        robot.stop();
                        robot.setDriftMode(false);


                        robot.drive(0,-1 * (isBlueAlliance?1:-1), 0);
                        sleep(750);
                        robot.stop();
                    }

                } else { // if doesnt sense anythig do park code

                    sleep(500);
                    robot.drive(0,-1 * (isBlueAlliance?1:-1), 0);
                    sleep(1100);
                    robot.stop();
                }

                break;

            case 4: //Retract jewelArm

                robot.jewelArm.setPower(1);
                if(robot.jewelArm.getCurrentPosition() <= 0){
                    robot.jewelArm.stop();
                    state++;
                }

                break;

            case 5: //
                break;

        }
    }
}
