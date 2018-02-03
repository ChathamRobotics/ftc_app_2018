package org.firstinspires.ftc.team9853.opmodes.tests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.chathamrobotics.common.Controller;
import org.chathamrobotics.common.opmode.TeleOpTemplate;
import org.firstinspires.ftc.team9853.Robot9853;
import org.firstinspires.ftc.team9853.opmodes.Tele9853;

import java.util.ArrayList;
import java.util.Map;

/**
 * Created by carsonstorm on 11/16/2017.
 */

@TeleOp(name = "Color Test", group = "Test")
public class ColorTest extends OpMode {
    private ArrayList<Map.Entry<String, ColorSensor>> colorSensors;
    private int i = 0;
    private Controller controller1;

    @Override
    public void init() {
        int address = 0x3c;
        colorSensors = new ArrayList<>(hardwareMap.colorSensor.entrySet());
        controller1 = new Controller(gamepad1);

        for (Map.Entry<String, ColorSensor> entry : colorSensors) {
            entry.getValue().setI2cAddress(I2cAddr.create8bit(address));
            address += 2;
        }

        telemetry.addLine("Found " + colorSensors.size() + " sensors");
    }

    @Override
    public void loop() {
        controller1.update();

        if (controller1.aState == Controller.ButtonState.TAPPED)
            i = i >= colorSensors.size() - 1 ? 0 : i + 1;
        if (controller1.bState == Controller.ButtonState.TAPPED)
            i = i <= 0 ? colorSensors.size() - 1 : i - 1;

        Map.Entry<String, ColorSensor> currentEntry = colorSensors.get(i);
        ColorSensor sensor = currentEntry.getValue();
        String name = currentEntry.getKey();

        if (controller1.xState == Controller.ButtonState.TAPPED)
            sensor.enableLed(true);
        if (controller1.yState == Controller.ButtonState.TAPPED)
            sensor.enableLed(false);

        telemetry.addLine("Data for " + name + ":");
        telemetry.addData("Address", Integer.toHexString(sensor.getI2cAddress().get8Bit()));
        telemetry.addData("argb", sensor.argb());
        telemetry.addData("alpha", sensor.alpha());
        telemetry.addData("red", sensor.red());
        telemetry.addData("green", sensor.green());
        telemetry.addData("blue", sensor.blue());

        if (sensor instanceof ModernRoboticsI2cColorSensor) {
            telemetry.addData("color number", ((ModernRoboticsI2cColorSensor) sensor).readUnsignedByte(ModernRoboticsI2cColorSensor.Register.COLOR_NUMBER));
        }
    }
}
