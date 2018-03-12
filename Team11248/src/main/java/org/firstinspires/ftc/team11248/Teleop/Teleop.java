package org.firstinspires.ftc.team11248.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.team11248.Hardware.Claw;
import org.firstinspires.ftc.team11248.Hardware.HolonomicDriver_11248;
import org.firstinspires.ftc.team11248.RevRobot;

import static org.firstinspires.ftc.team11248.Teleop.Teleop.ENCODER_THRESHOLD;
import static org.firstinspires.ftc.team11248.Teleop.Teleop.killed;

/**
 * Created by Tony_Air on 11/6/17.
 */


@TeleOp(name="Drive")
public class Teleop extends OpMode {

    private final boolean IS_COMP = true;

    static final int ENCODER_THRESHOLD = 50; //Thread
    static volatile boolean killed = false; //Thread

    RevRobot robot;
    private Gamepad prevGP1, prevGP2;

    @Override
    public void init() {

        robot = new RevRobot(hardwareMap, telemetry);
        robot.init_teleop();

        prevGP1 = new Gamepad();
        prevGP2 = new Gamepad();

        try {
            prevGP1.copy(gamepad1);
            prevGP2.copy(gamepad2);
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }

        robot.setOffsetAngle(0);
        robot.jewelArm.enableColorLed(false);

    }



    @Override
    public void loop() {

        if(IS_COMP) robot.printTeleOpTelemetry();
        else robot.printTelemetry();

        /*
        Direction Swap
         */

        Claw GP1_Claw = (robot.getOffsetAngle() == 0) ? robot.frontClaw : robot.backClaw;
        Claw GP2_Claw = (robot.getOffsetAngle() == 0) ? robot.backClaw : robot.frontClaw;

        if (gamepad1.dpad_up) {
            robot.setOffsetAngle(0);
            robot.frontClaw.open();
            robot.backClaw.release();

        } else if (gamepad1.dpad_down) {
            robot.setOffsetAngle(HolonomicDriver_11248.BACK_OFFSET);
            robot.backClaw.open();
            robot.frontClaw.release();

        } else if (gamepad1.dpad_left) robot.setOffsetAngle(HolonomicDriver_11248.LEFT_OFFSET); // not used
        else if (gamepad1.dpad_right) robot.setOffsetAngle(HolonomicDriver_11248.RIGHT_OFFSET);




        /*
         *              GAMEPAD 1
         */


        /*
        Drive
         */

        robot.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, true);
        robot.setFastMode(gamepad1.right_bumper);
        if(gamepad1.y && !prevGP1.y) robot.toggleSlowMode();


        /*
        Active Claw
         */

        if(gamepad1.a && !prevGP1.a){ //Top

            if(GP1_Claw.topState == Claw.Position.OPEN) GP1_Claw.grabTop(); // Grab if open (Can go from closed to grab)
            else GP1_Claw.openTop(); // if claw is closed or grabbed then open

        }

        if(gamepad1.b && !prevGP1.b){ //Bottom

            if(GP1_Claw.bottomState == Claw.Position.OPEN) GP1_Claw.grabBottom(); // Grab if open (Can go from closed to grab)
            else GP1_Claw.openBottom(); // if claw is closed or grabbed then open
        }

        if(gamepad1.x && !prevGP1.x){

            if(GP1_Claw.topState == Claw.Position.RELEASE) GP1_Claw.open(); // Grab if open (Can go from closed to grab)
            else GP1_Claw.release(); // if claw is closed or grabbed then open

        }

        if(GP1_Claw.runMode != DcMotor.RunMode.RUN_WITHOUT_ENCODER){ //if in encoder mode check if were trying to move
            if (gamepad1.left_trigger > 0 || gamepad1.right_trigger > 0) GP1_Claw.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }else { // if not in encoder mode, continue normally
            if (gamepad1.right_trigger > 0) {
                GP1_Claw.setPower(gamepad1.right_trigger);
                if (GP1_Claw.runMode != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                    GP1_Claw.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else if (gamepad1.left_trigger > 0) {
                GP1_Claw.setPower(-gamepad1.left_trigger);
                if (GP1_Claw.runMode != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                    GP1_Claw.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else GP1_Claw.setPower(0);
        }

        if(gamepad1.left_bumper) GP1_Claw.moveToPosition(Claw.UP1);




        /*
         *              GAMEPAD 2
         */


        /*
        Relic Arm
         */

        if(gamepad2.a) robot.relicArm.grab(); //does not need debouncing
        if(gamepad2.y) robot.relicArm.up();
        if(gamepad2.b) robot.relicArm.release();

        if (gamepad2.right_trigger > 0) robot.relicArm.setPower(gamepad2.right_trigger);
        else if (gamepad2.left_trigger > 0) robot.relicArm.setPower(-gamepad2.left_trigger);
        else robot.relicArm.setPower(0);


        /*
        Jewel Arm
         */

        if(gamepad2.right_bumper) robot.jewelArm.setPower(1);
        else if (gamepad2.left_bumper) robot.jewelArm.setPower(-1);
        else robot.jewelArm.setPower(0);


        /*
        Passive Claw
         */

        if(GP2_Claw.runMode != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
            if (Math.abs(gamepad2.left_stick_y)>0) GP2_Claw.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }else GP2_Claw.setPower(-gamepad2.left_stick_y);



        if(gamepad2.dpad_down && !prevGP2.dpad_down){

            if(GP2_Claw.topState == Claw.Position.OPEN && GP2_Claw.bottomState == Claw.Position.OPEN) GP2_Claw.grab();
            else GP2_Claw.open();

        }

         /*
        Balancing Stone
         */

        if(gamepad2.x) {
            robot.frontClaw.moveToPosition(Claw.UP_BALANCE);
            robot.backClaw.moveToPosition(Claw.UP_BALANCE);

        }




        /*
            Recapture all previous values from Gamepads for debouncing
         */
        try {
            prevGP1.copy(gamepad1);
        } catch (RobotCoreException e1) {
            e1.printStackTrace();
        }

        try {
            prevGP2.copy(gamepad2);
        } catch (RobotCoreException e1) {
            e1.printStackTrace();
        }
    }

    @Override
    public void stop() {
        killed = true;
    }
}


class MoveLift extends Thread{

    private Claw claw;
    private int targetPosition;

    MoveLift(Claw claw, int targetPosition){
       this.claw = claw;
       this.targetPosition = targetPosition;
    }

    private boolean isAtPosition() {

        int currentPos = Math.abs(claw.getCurrentPosition());
        return currentPos <= (targetPosition + ENCODER_THRESHOLD) || (currentPos - ENCODER_THRESHOLD) >= targetPosition;
    }

    @Override
    public void run() {

        claw.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        claw.setTargetPosition(targetPosition);

        int currentPos = claw.getCurrentPosition();
        int distToTarget = targetPosition - currentPos + currentPos<0?Math.abs(currentPos):0;

        claw.setPower( distToTarget/Math.abs(distToTarget) ); //-1 or 1

        while (!isInterrupted() && !killed && !isAtPosition());

        claw.setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // claw.setPower( .75 * distToTarget/(Claw.MAX_ENCODER_COUNT/2) + .25 * distToTarget/Math.abs(distToTarget) );

    }
}

