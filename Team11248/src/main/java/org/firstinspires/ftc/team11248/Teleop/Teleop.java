package org.firstinspires.ftc.team11248.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.team11248.Hardware.Claw;
import org.firstinspires.ftc.team11248.Hardware.HolonomicDriver_11248;
import org.firstinspires.ftc.team11248.RevRobot;

/**
 * Created by Tony_Air on 11/6/17.
 */


@TeleOp(name="Drive")
public class Teleop extends OpMode {

    private final boolean IS_COMP = false;

    RevRobot robot;
    private Gamepad prevGP1, prevGP2;

    @Override
    public void init() {

        robot = new RevRobot(hardwareMap, telemetry);
        robot.init();
        robot.relicArm.up();

        prevGP1 = new Gamepad();
        prevGP2 = new Gamepad();

        try {
            prevGP1.copy(gamepad1);
            prevGP2.copy(gamepad2);
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }

        robot.setOffsetAngle(0);
    }



    @Override
    public void loop() {

        if(IS_COMP) robot.printCompTelemetry();
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

        } else if (gamepad1.dpad_left) {
            robot.setOffsetAngle(HolonomicDriver_11248.LEFT_OFFSET);
            robot.backClaw.open();
            robot.frontClaw.release();

        } else if (gamepad1.dpad_right) {
                robot.setOffsetAngle(HolonomicDriver_11248.RIGHT_OFFSET);
                robot.backClaw.open();
                robot.frontClaw.release();

        }



        /*
         *              GAMEPAD 1
         */


        /*
        Drive
         */


        robot.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, true);
        robot.setFastMode(gamepad1.right_bumper);


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

        if (gamepad1.right_trigger > 0) GP1_Claw.setPower(gamepad1.right_trigger);
        else if (gamepad1.left_trigger > 0) GP1_Claw.setPower(-gamepad1.left_trigger);
        else GP1_Claw.setPower(0);

        if(gamepad1.y && !prevGP1.y) robot.toggleSlowMode();




        /*
         *              GAMEPAD 2
         */


        /*
        Relic Arm
         */

        if(gamepad2.a && !prevGP2.a) robot.relicArm.grab();
        if(gamepad2.y && !prevGP2.y) robot.relicArm.up();
        if(gamepad2.b && !prevGP2.b) robot.relicArm.release();

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

        GP2_Claw.setPower(-gamepad2.left_stick_y);

        if(gamepad2.dpad_down && !prevGP2.dpad_down){

            if(GP2_Claw.topState == Claw.Position.OPEN && GP2_Claw.bottomState == Claw.Position.OPEN) GP2_Claw.grab();
            else GP2_Claw.open();

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
}


