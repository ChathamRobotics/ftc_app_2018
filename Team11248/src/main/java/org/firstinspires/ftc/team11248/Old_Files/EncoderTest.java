package org.firstinspires.ftc.team11248.Old_Files;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.team11248.Old_Files.Robot11248;

/**
 * Created by Tony_Air on 11/26/17.
 */
@TeleOp(name = "Encoder Test")
@Disabled
public class EncoderTest extends OpMode {

    Robot11248 robot;

    final int TICKS_PER_REVOLUTION = 1120;

    @Override
    public void init() {
        robot = new Robot11248(hardwareMap, telemetry);

        robot.init();
        robot.deactivateColorSensors();

        robot.frontLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        robot.frontLift.setTargetPosition(TICKS_PER_REVOLUTION);
        robot.frontLift.setPower(.25);

        telemetry.addData("01:", "Ticks: " + robot.frontLift.getCurrentPosition());

    }
}
