package org.firstinspires.ftc.team11248.Autonomous;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.team11248.Hardware.Claw;
import org.firstinspires.ftc.team11248.RevRobot;

/**
 * Created by tonytesoriero on 2/19/18.
 */

@Autonomous(name="brick")
public class Brick_Auto extends LinearOpMode{

    private final boolean IS_COMP = true;
    private final int STOP_DELAY = 500;
    private final int DEPOSIT_DELAY = 500;

    private int targetAngle = 15; //Left Center or Unknown

    private boolean isBlueAlliance = true;

    private RevRobot robot;
    private Claw claw, passiveClaw;

    private int state = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        /*
        Inits
         */

        ElapsedTime mRuntime = new ElapsedTime();
        ElapsedTime vuforiaLoopTimer = new ElapsedTime();


        robot = new RevRobot(hardwareMap, telemetry);
        robot.init();


        //with !isBlueAlliance pushes glyph (front claw)
        claw = !isBlueAlliance ? robot.frontClaw : robot.backClaw;
        passiveClaw = isBlueAlliance ? robot.frontClaw : robot.backClaw;


        /*
        Wait for start
         */

        while(!isStarted()){
            robot.vuforia.update();
            robot.printAlignmentTelemetry();
            robot.printAutoTelemetry();
            telemetry.update();
        }

        robot.relicArm.up();
        sleep(200);
        /*
        Set baselines
         */

        mRuntime.reset();
        robot.resetEncoders();
        robot.imu.setBaseline();


        /*
        Grab Glyph
         */

        claw.grabTop();
        sleep(200); //wait for servo grab



        while (opModeIsActive() && !isStopRequested() && state < 12 && mRuntime.milliseconds()< 29000) {

            if(!IS_COMP) mRuntime.reset();


            /*
            Print telemetry
             */

            telemetry.addData("Auto", "State: " + state);


            /*

             */

            robot.vuforia.update();
            robot.imu.logHeadingChanges();

            switch (state) {

                case 0: //Pick up Glyph and start first jewelArm move

                    claw.setPower(1);

                    if (claw.getCurrentPosition() >= Claw.PICK_UP_GLYPH) {
                        claw.stop();

                        robot.jewelArm.stop();
                        sleep(STOP_DELAY);
                        robot.jewelArm.setBaseLine();
                        robot.vuforia.init(true, !IS_COMP);
                        state=7;
                    }

                    break;


                case 1: //Extend jewelArm till hits wall or maxes out

                    robot.jewelArm.setPower(.35);
                    robot.jewelArm.updateCache();

                    if (robot.jewelArm.pressed()) { //hits wall, record position and back up
                        robot.jewelArm.stop();
                        robot.jewelArm.rotationsToWall = robot.jewelArm.getCurrentPosition();
                        sleep(STOP_DELAY);
                        state++;

                    } else if (robot.jewelArm.getCurrentPosition() >= robot.jewelArm.MAX_ENCODER_COUNT) { // doesnt, stop and try and read color
                        robot.jewelArm.stop();
                        sleep(STOP_DELAY);
                        state += 2;
                    }
                    break;


                case 2: //Back up to read jewel

                    robot.jewelArm.setPower(-.35);
                    robot.jewelArm.updateCache();

                    int targetPosition = robot.jewelArm.rotationsToWall - robot.jewelArm.BACK_UP_ROTATIONS;

                    if (robot.jewelArm.getCurrentPosition() <= targetPosition) {
                        robot.jewelArm.stop();
                        sleep(STOP_DELAY);
                        state++;
                    }

                    break;


                case 3: //Read jewel color

                    //if sees one, check if its lft right
                    //if nothing go to state with moving the arm back in

                    boolean isLeftJewelRed = false;
                    boolean isLeftJewelBlue = false;

                    if(robot.jewelDetector.getLastOrder() != JewelDetector.JewelOrder.UNKNOWN){
                        isLeftJewelRed = robot.jewelDetector.getLastOrder() == JewelDetector.JewelOrder.RED_BLUE;
                        isLeftJewelBlue = robot.jewelDetector.getLastOrder() == JewelDetector.JewelOrder.BLUE_RED;

                    } else {
                        isLeftJewelRed = robot.jewelArm.redCache;
                        isLeftJewelBlue = robot.jewelArm.blueCache;

                    }

                    if (isLeftJewelBlue != isLeftJewelRed) {


                        if (isLeftJewelBlue) { //if in direction of cryptobox, drive off to hit jewel

                            fallOffBuildPlate();
                            state += 3;


                        } else { //if in opposite, rotate to hit jewel

                            robot.drive(0, 0, isBlueAlliance ? -.75 : .75);
                            sleep(500);
                            robot.stop();

                            state += 2;

                        }

                    } else {

                        state ++; // if doesnt sense anything retract arm and continue
                    }
                    break;


                case 4:// case where it doesnt see anyhting (Retract jewel arm and fall)

                    robot.jewelArm.setPower(-1);
                    if (robot.jewelArm.getCurrentPosition() <= 0) {

                        robot.jewelArm.stop();
                        fallOffBuildPlate();
                        vuforiaLoopTimer.reset();
                        state += 3;
                    }
                    break;


                case 5: // case where it rotates to hit jewel (retract jewel arm rotate robot back and fall off build plate)

                    robot.jewelArm.setPower(-1);
                    if (robot.jewelArm.getCurrentPosition() <= 0) {

                        robot.jewelArm.stop();

                        robot.drive(0, 0, isBlueAlliance ? .75 : -.75);
                        sleep(500);
                        robot.stop();
                        fallOffBuildPlate();
                        vuforiaLoopTimer.reset();
                        state += 2;
                    }
                    break;


                case 6: //Case where it falls off build plate and hits jewel (Retract jewelArm)

                    robot.jewelArm.setPower(-1);
                    if (robot.jewelArm.getCurrentPosition() <= 0) {
                        robot.jewelArm.stop();
                        vuforiaLoopTimer.reset();
                        state++;
                    }
                    break;



                case 7: //wait to read vumark

                    if(vuforiaLoopTimer.milliseconds() > 2000){

                        state++;
                    }


                    break;


                case 8: //Drive off build plate till flat

                    robot.drive(0, -.5 * (isBlueAlliance ? 1 : -1), 0);

                    if (robot.imu.isAtAngle('Y', 0, .5)) {
                        robot.stop();
                        sleep(STOP_DELAY);
                        robot.resetDriveEncoders();

                        state++;
                    }

                    break;


                case 9: // drive forward to align with cryptobox center //NOTE uncomment for front

//                    int targetRotations = 100; //default Center or Unknown
//
////                    if(robot.vuforia.getLastImage() == RelicRecoveryVuMark.LEFT ) targetRotations = 0;
////                    else if(robot.vuforia.getLastImage() == RelicRecoveryVuMark.RIGHT ) targetRotations = 200;
//
//                    robot.drive(0, -.5 * (isBlueAlliance ? 1 : -1), 0); //drive into box
//
//                    if(Math.abs(robot.getCurrentPosition()) >= targetRotations){
                    robot.vuforia.deactivateTracking(); //locks lastImage
                    state++;
//                    }


                    break;


                case 10:// align robot angle to cryptobox

                    if(robot.vuforia.getLastImage() == RelicRecoveryVuMark.CENTER ) targetAngle = 30;
                    else if(robot.vuforia.getLastImage() == RelicRecoveryVuMark.LEFT ) targetAngle = 45;

                    if ( robot.moveToAngle( ((isBlueAlliance?-1:1)) * targetAngle) ) state++;
                    break;


                case 11: //push glyph in cryptobox

                    depositGlyphs(claw, isBlueAlliance);

                    sleep(STOP_DELAY);

                    robot.drive(0, (isBlueAlliance ? 1 : -1), 0);//reverse to box

                    state++;
                    break;


//                case 12: // align to glyph pit to attempt 3 glyph auto
//
//                    if ( robot.moveToAngle(0 )){
//                        passiveClaw.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
//                        passiveClaw.setTargetPosition(Claw.UP1);
//                        state++;
//                    }
//                    break;
//
//
//                case 13: // pick up back claw for 2 glyph grab
//
//                    passiveClaw.setPower(1);
//
//                    if(passiveClaw.getCurrentPosition() >= Claw.UP1){
//                        passiveClaw.setPower(0);
//                        passiveClaw.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                        state++;
//                    }
//                    break;
//
//
//                case 14: // drive into glyph pit and grab 2 new blocks
//
//                    grabNewGlyphs();
//                    sleep(STOP_DELAY);
//
//                    state++;
//                    break;
//
//
//                case 15:// realign to box
//
//                    if ( robot.moveToAngle( ((isBlueAlliance?-1:1)) * targetAngle) ) state++;
//                    break;
//
//
//                case 16://deposit glyphs
//
//                    depositGlyphs(passiveClaw, !isBlueAlliance);
//
//                    robot.drive(0, -1 * (isBlueAlliance ? 1 : -1), 0);//back up for park
//                    sleep(700);
//                    robot.stop();
//
//                    state++;
//                    break;

            }//switch

        }//loop 1


        robot.stop();

        while (opModeIsActive() && !isStopRequested()){  //reset claws to 0 for teleop encoder functions to work

            if(robot.frontClaw.getCurrentPosition() <= 0) robot.frontClaw.setPower(0);
            else robot.frontClaw.setPower(-1);

            if(robot.backClaw.getCurrentPosition() <= 0) robot.backClaw.setPower(0);
            else robot.backClaw.setPower(-1);

            if(robot.jewelArm.getCurrentPosition() <= 0) robot.jewelArm.setPower(0);
            else robot.jewelArm.setPower(-1);

            if(robot.frontClaw.getCurrentPosition() <= 0 && robot.backClaw.getCurrentPosition() <= 0 && robot.jewelArm.getCurrentPosition() <= 0) {
                break;
            }

        } //loop 2


    }//class

    private void fallOffBuildPlate(){

        robot.drive(0, .5 * (isBlueAlliance? -1 : 1), 0);
        sleep(250);//to drive
        robot.setDriftMode(true);

        robot.stop();
        sleep(500);//to fall
        robot.setDriftMode(false);
    }

    private void depositGlyphs(Claw claw, Boolean direction){
        robot.drive(0, -1 * (isBlueAlliance ? 1 : -1), 0); //drive into box
        sleep(700);
        robot.stop();

        claw.open();

        sleep(DEPOSIT_DELAY);

        robot.drive(0, .5 * (direction ? 1 : -1), 0); //back up
        sleep(500);
        robot.stop();

        sleep(DEPOSIT_DELAY);

        robot.drive(0, -.5 * (direction ? 1 : -1), 0);//tap againg
        sleep(750);
        robot.stop();

        sleep(DEPOSIT_DELAY);

        robot.drive(0, .5 * (direction ? 1 : -1), 0);//back up
        sleep(750);
        robot.stop();

    }

    private void grabNewGlyphs(){

        robot.setFastMode(true);
        robot.drive(0, 1.0 * (isBlueAlliance ? 1 : -1), 0); // drive into pit
        sleep(700);
        robot.stop();
        robot.setFastMode(false);

        sleep(STOP_DELAY);

        passiveClaw.grab(); //grab
        sleep(200);

        passiveClaw.setPower(1);//raise glyphs
        sleep(300);
        passiveClaw.setPower(0);


        robot.setFastMode(true);
        robot.drive(0, -1 * (isBlueAlliance ? 1 : -1), 0);//reverse to box
        sleep(700);
        robot.stop();
        robot.setFastMode(false);
    }

}
