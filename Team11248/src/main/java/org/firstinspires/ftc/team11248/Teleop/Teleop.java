package org.firstinspires.ftc.team11248.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.team11248.Hardware.Claw;
import org.firstinspires.ftc.team11248.Hardware.HolonomicDriver_11248;
import org.firstinspires.ftc.team11248.Robot11248;
import org.firstinspires.ftc.team11248.States_Robot.RevRobot;

/**
 * Created by Tony_Air on 11/6/17.
 */


@TeleOp(name="Drive")
public class Teleop extends OpMode {

    RevRobot robot;
    private Gamepad prevGP1, prevGP2;

    @Override
    public void init() {

        robot = new RevRobot(hardwareMap, telemetry);
        robot.init();

        prevGP1 = new Gamepad();
        prevGP2 = new Gamepad();

        try {
            prevGP1.copy(gamepad1);
            prevGP2.copy(gamepad2);
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }

        robot.setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.setOffsetAngle(0);
    }



    @Override
    public void loop() {

        /*
        DRIVE
         */

        robot.drive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, true);
        if (gamepad1.y && !prevGP1.y) robot.toggleDriftMode();
        robot.setFastMode(gamepad1.right_bumper);

        //DIRECTION SWAP
        if (gamepad1.dpad_up) {
            robot.setOffsetAngle(0);
            robot.frontClaw.open();
            robot.backClaw.release();

        } else if (gamepad1.dpad_down) {
            robot.setOffsetAngle(HolonomicDriver_11248.BACK_OFFSET);
            robot.backClaw.open();
            robot.frontClaw.release();

        }




        /*
        RELIC
         */
        robot.relicArm.setPower(gamepad2.left_stick_y);

        if (gamepad2.dpad_up) robot.relicArm.up();
        if (gamepad2.dpad_down) robot.relicArm.grab();
        if (gamepad2.b) robot.relicArm.release();



        /*
        CLAW
         */
        Gamepad frontLiftGamepad = (robot.getOffsetAngle() == 0) ? gamepad1 : gamepad2;
        Gamepad backLiftGamepad = (robot.getOffsetAngle() == 0) ? gamepad2 : gamepad1;

        Gamepad frontLiftGamepadPrev = (robot.getOffsetAngle() == 0) ? prevGP1 : prevGP2;
        Gamepad backLiftGamepadPrev = (robot.getOffsetAngle() == 0) ? prevGP2 : prevGP1;



        // Front lift controls

        if (frontLiftGamepad.a && !frontLiftGamepadPrev.a) {

            if (robot.frontClaw.state == Claw.Position.OPEN) // Grab if open (Can go from closed to grab)
                robot.frontClaw.grab();

            else  // if claw is closed or grabbed then open
                robot.frontClaw.open();
        }

        if (frontLiftGamepad.b && !frontLiftGamepadPrev.b){
            robot.frontClaw.grabTop();
        }

        if (frontLiftGamepad.x && !frontLiftGamepadPrev.x)
            if (robot.frontClaw.state == Claw.Position.RELEASE) {
                robot.frontClaw.open();

            } else {  // if claw is open or grabbed then close
                robot.frontClaw.release();
            }

        //Sets arm motor to whatever right trigger is
        if (frontLiftGamepad.right_trigger > 0)
            robot.frontClaw.setPower(frontLiftGamepad.right_trigger);
        else if (frontLiftGamepad.left_trigger > 0)
            robot.frontClaw.setPower(-frontLiftGamepad.left_trigger);
        else
            robot.frontClaw.setPower(0);


        // Back lift controls

        if (backLiftGamepad.a && !backLiftGamepadPrev.a) {

            if (robot.backClaw.state == Claw.Position.OPEN) // Grab if open (Can go from closed to grab)
                robot.backClaw.grab();

            else  // if claw is closed or grabbed then open
                robot.backClaw.open();
        }

        if (backLiftGamepad.x && !backLiftGamepadPrev.x)
            if (robot.backClaw.state == Claw.Position.RELEASE) {
                robot.backClaw.open();

            } else {  // if claw is open or grabbed then close
                robot.backClaw.release();
            }

        //Sets arm motor to whatever right trigger is
        if (backLiftGamepad.right_trigger > 0)
            robot.backClaw.setPower(backLiftGamepad.right_trigger);

        else if (backLiftGamepad.left_trigger > 0)
            robot.backClaw.setPower(-backLiftGamepad.left_trigger);

        else
            robot.backClaw.setPower(0);




            //Recaptures all previous values of Gamepad 1 for debouncing
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


