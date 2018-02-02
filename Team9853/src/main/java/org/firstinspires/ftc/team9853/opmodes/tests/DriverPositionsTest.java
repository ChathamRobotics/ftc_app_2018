package org.firstinspires.ftc.team9853.opmodes.tests;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.chathamrobotics.common.Controller;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.team9853.opmodes.Tele9853;

/**
 * Created by carsonstorm on 2/1/2018.
 */

@TeleOp(name = "Driver Position Test", group = "TEST")
public class DriverPositionsTest extends Tele9853 {
    int position;
    double power;

    @Override
    public void init() {
        super.init();
        robot.driver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driver.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        if (controller1.aState == Controller.ButtonState.TAPPED) position+= 50;
        if (controller1.bState == Controller.ButtonState.TAPPED) position-=50;

        if (controller1.xState == Controller.ButtonState.TAPPED) power+=.1;
        if (controller1.yState == Controller.ButtonState.TAPPED) power-=.1;

        robot.log.info("Target position", position);
        robot.log.info("Power", power);

        robot.driver.setTargetPosition(position);
        robot.driver.setDrivePower(90, AngleUnit.DEGREES, Range.clip(power, -1, 1), 0);
    }
}
