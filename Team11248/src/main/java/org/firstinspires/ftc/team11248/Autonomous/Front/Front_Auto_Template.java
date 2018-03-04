package org.firstinspires.ftc.team11248.Autonomous.Front;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team11248.Hardware.Claw;
import org.firstinspires.ftc.team11248.RevRobot;

/**
 * Created by tonytesoriero on 2/19/18.
 */

public class Front_Auto_Template extends LinearOpMode{

    private final boolean IS_COMP = false;
    private final int STOP_DELAY = 500;

    private boolean isBlueAlliance;

    private RevRobot robot;
    private Claw claw;

    private int state = 0;

    Front_Auto_Template(boolean isBlueAlliance){
        this.isBlueAlliance = isBlueAlliance;

    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RevRobot(hardwareMap, telemetry);
        robot.init();
        robot.init_IMU();

        claw = !isBlueAlliance ? robot.frontClaw : robot.backClaw; // with ! drags glyph (back claw)

        robot.vuforia.init(true, IS_COMP);
        robot.vuforia.activateTracking();


        waitForStart();

        robot.resetEncoders();
        robot.setIMUBaseline();
        robot.relicArm.up();


        claw.grabTop();
        sleep(200); //wait for servo grab

        switch (state){

            case 0: //Pick up Glyph and start first jewelArm move

                claw.setPower(1);

                if(claw.getCurrentPosition() >= claw.PICK_UP_GLYPH){ //TODO: pick up glyph
                    claw.setPower(0);

                    robot.jewelArm.setPower(1);
                    sleep(1000);
                    robot.jewelArm.setBaseLine();
                    state++;
                }

                break;


            case 1: //Extend jewelArm till hits wall or maxes out

                robot.jewelArm.setPower(.25);

                if(robot.jewelArm.pressed() ){ //hits wall, record position and back up
                    robot.jewelArm.stop();
                    robot.jewelArm.rotationsToWall = robot.jewelArm.getCurrentPosition();
                    sleep(STOP_DELAY);
                    state++;
                }

                if (robot.jewelArm.getCurrentPosition() >= robot.jewelArm.MAX_ENCODER_COUNT){ // doesnt, stop and try and read color
                    robot.jewelArm.stop();
                    sleep(STOP_DELAY);
                    state += 2;
                }
                break;


            case 2: //Back up to read jewel

                robot.jewelArm.setPower(-.25);
                int targetPosition = robot.jewelArm.rotationsToWall - robot.jewelArm.BACK_UP_ROTATIONS;

                if (robot.jewelArm.getCurrentPosition() <= targetPosition ){
                    robot.jewelArm.stop();
                    sleep(STOP_DELAY);
                    state++;
                }

                break;


            case 3: //Read jewel color

                //if sees one, chck if its lft right
                //if nothing go to state with moving the arm back in

                boolean isLeftJewelRed = robot.jewelArm.isRed();//TODO: test isBlue isRed
                boolean isLeftJewelBlue = robot.jewelArm.isBlue();

                if( !(isLeftJewelBlue == isLeftJewelRed) ) {


                    if(isLeftJewelBlue){ //if in direction of cryptobox, drive off to hit jewel

                        fallOffBuildPlate();
                        state += 2;


                    } else { //if in opposite, rotate to hit jewel

                        robot.drive(0, 0, isBlueAlliance? -1 : 1);
                        sleep(500);
                        robot.stop();

                        state++;

                    }

                } else state += 2; // if doesnt sense anything retract arm and continue
                break;


            case 4: // case to retract jewel arm rotate robot back and fall off build plate

                robot.jewelArm.setPower(1);
                if(robot.jewelArm.getCurrentPosition() <= 0){

                    robot.jewelArm.stop();

                    robot.drive(0, 0, isBlueAlliance? 1 : -1);
                    sleep(500);
                    robot.stop();
                    
                    fallOffBuildPlate();

                    state += 2;
                }
                break;


            case 5: //Retract jewelArm

                robot.jewelArm.setPower(1);
                if(robot.jewelArm.getCurrentPosition() <= 0){
                    robot.jewelArm.stop();
                    state++;
                }
                break;


            case 6: //Drive off build plate till flat

                robot.drive(0,-.5 * (isBlueAlliance?1:-1), 0);

                if(robot.isAtAngle('Y', robot.IMUBaseline[1], 5)){ //TODO
                    robot.stop();
                    sleep(STOP_DELAY);
                    state++;
                }
                break;


            case 7:// align robot flat

                if( robot.moveToAngle(robot.IMUBaseline[0]) )

                break;

            case 8: //parking zone

                robot.drive(0,-1 * (isBlueAlliance?1:-1), 0);
                sleep(1100);
                robot.stop();
                break;
        }
    }

    void fallOffBuildPlate(){

        robot.drive(0, .5 * (isBlueAlliance? -1 : 1), 0);
        sleep(500);//to drive
        robot.setDriftMode(true);

        robot.stop();
        sleep(500);//to fall
        robot.setDriftMode(false);
    }
}
