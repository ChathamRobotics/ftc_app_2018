package org.firstinspires.ftc.team11248.Hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.chathamrobotics.common.hardware.utils.HardwareListener;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Jewel_Arm {

    private DcMotor motor;
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private DigitalChannel touchSensor;
    private Telemetry telemetry;

    public Jewel_Arm (String motor, String colorSensor, String distanceSensor, String touchSensor, HardwareMap hardwareMap, Telemetry telemetry){

        this.motor = hardwareMap.dcMotor.get(motor);
        this.colorSensor = hardwareMap.colorSensor.get(colorSensor);
        this.distanceSensor = hardwareMap.get(DistanceSensor.class, distanceSensor);
        this.touchSensor = hardwareMap.digitalChannel.get(touchSensor);
        this.telemetry = telemetry;
    }


    public boolean isRed(){

        return (colorSensor.red()>colorSensor.blue());

    }

    public void isBlue(){

    }


}
