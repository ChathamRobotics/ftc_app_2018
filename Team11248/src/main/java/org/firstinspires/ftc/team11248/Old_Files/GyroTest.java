package org.firstinspires.ftc.team11248.Old_Files;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


import org.firstinspires.ftc.team11248.Old_Files.Robot11248;

/**
 * Created by Tony_Air on 11/26/17.
 */
@TeleOp(name = "Gyro Test")
@Disabled
public class GyroTest extends OpMode {

    Robot11248 robot;

    @Override
    public void init() {
        robot = new Robot11248(hardwareMap, telemetry);

        robot.init();
        robot.deactivateColorSensors();
        robot.calibrateGyro();
        robot.activateColorSensors();
    }

    @Override
    public void loop() {

        robot.moveToAngle(170);

        telemetry.addData("01: ", "Heading: " + robot.getGyroAngle());
    }
}
