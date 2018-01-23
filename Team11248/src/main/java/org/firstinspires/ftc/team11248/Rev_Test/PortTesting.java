package org.firstinspires.ftc.team11248.Rev_Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.team11248.Hardware.MRRangeSensor_V2;


@TeleOp(name = "REV TEST")
public class PortTesting extends OpMode{

    DcMotor motor;
    Servo servo;
    MRRangeSensor_V2 range;
    byte SensorAddress = 0x28;
    TouchSensor touch;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        range = new MRRangeSensor_V2(hardwareMap.get("range"), SensorAddress);

        touch = hardwareMap.get(TouchSensor.class, "touch");

        servo = hardwareMap.get(Servo.class, "servo");
    }

    @Override
    public void loop() {

        double motorPower = 0;

        if (gamepad1.right_trigger > 0)
           motorPower = gamepad1.right_trigger;

        else if (gamepad1.left_trigger > 0)
            motorPower = -gamepad1.left_trigger;

        motor.setPower(motorPower);

        servo.setPosition((gamepad1.a)?0:1);


        telemetry.addData("01:", "Motor Power: " + motorPower);
        telemetry.addData("02:", "Encoder Ticks: " + motor.getCurrentPosition());
        telemetry.addData("03:", "Touch Sensor: " + touch.isPressed());
//        telemetry.addData("04:", "Range Sensor: " + range.ultrasonicValue());


    }
}
