package org.firstinspires.ftc.team11248.Old_Files.CompSci_Education;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Tony_Air on 12/18/17.
 */


@TeleOp(name = "Beatle")
@Disabled
public class BeatleTele extends OpMode {

    Beatle robot;

    @Override
    public void init() {

        robot = new Beatle(hardwareMap, telemetry);
        robot.init();
    }

    @Override
    public void loop() {

        robot.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        if(gamepad1.a) robot.toggle_claw();
    }
}
