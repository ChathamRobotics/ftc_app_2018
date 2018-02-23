package org.firstinspires.ftc.team11248.Hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.chathamrobotics.common.hardware.utils.HardwareListener;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Jewel_Arm {

    public final int MAX_ENCODER_COUNT = 2000;//TODO

    private final int COLOR_THRESHOLD = 30;//TODO
    private final int ENCODER_THRESHOLD = 5; //TODO

    private int lastRotation = 0;
    public int rotationsToWall;
    public final int BACK_UP_ROTATIONS = 200; //TODO

    public boolean redCache = false;
    public boolean blueCache = false;
    public boolean touchCache = false;

    public double blueBaseLine = 0;
    public double redBaseLine = 0;

    private DcMotor motor;
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private DigitalChannel touchSensor;
    private Telemetry telemetry;


    public Jewel_Arm (String motor, String colorDistanceSensor, String touchSensor, HardwareMap hardwareMap, Telemetry telemetry){

        this.motor = hardwareMap.dcMotor.get(motor);

        this.colorSensor = hardwareMap.colorSensor.get(colorDistanceSensor);
        this.distanceSensor = hardwareMap.get(DistanceSensor.class, colorDistanceSensor);

        this.touchSensor = hardwareMap.digitalChannel.get(touchSensor);
        this.touchSensor.setMode(DigitalChannel.Mode.INPUT);

        this.telemetry = telemetry;
    }

    public void init(){
        setDriftMode(false);
        resetEncoders();
        enableColorLed(true);
    }


    /*
    Motor Methods
     */

    public void setPower(double power){
        motor.setPower(power);
    }

    public void stop() { setPower(0); }

    public void setDriftMode(boolean on){
        motor.setZeroPowerBehavior( on? DcMotor.ZeroPowerBehavior.FLOAT:DcMotor.ZeroPowerBehavior.BRAKE );
    }

    public void setMotorMode(DcMotor.RunMode runmode){
        motor.setMode(runmode);
    }


    /*
    Encoder Methods
     */

    public void recordPosition(){
        lastRotation = motor.getCurrentPosition();
    }

    public int getCurrentPosition(){
        recordPosition();
        return lastRotation;
    }

    public int getLastPosition(){
        return lastRotation;
    }

    public void resetEncoders() {
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        recordPosition();
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTargetPosition(int target){
        motor.setTargetPosition(target);
    }

    public boolean isAtTargetPosition(){
        return ( getCurrentPosition() > (motor.getTargetPosition() + ENCODER_THRESHOLD) )
                || ( getCurrentPosition() < (motor.getTargetPosition() + ENCODER_THRESHOLD) );
    }


    /*
    Color Methods
     */

    public void setBaseLine(){
        blueBaseLine = colorSensor.blue();
        redBaseLine = colorSensor.red();
    }

    public boolean isBlue(){//could compaire base line to current blue

        double blue = Math.abs( colorSensor.blue() - blueBaseLine);
        double red =  Math.abs( colorSensor.red() - redBaseLine);

        blueCache = (blue > (red + COLOR_THRESHOLD));
        return blueCache;
    }

    public boolean isRed(){

        double blue = Math.abs( colorSensor.blue() - blueBaseLine);
        double red =  Math.abs( colorSensor.red() - redBaseLine);

        redCache = ((blue + COLOR_THRESHOLD) < red);
        return redCache;
    }

    public void enableColorLed(boolean on){
        colorSensor.enableLed(on);
    }

    /*
    Distance Methods
     */

    public double getDistance(){
        return distanceSensor.getDistance(DistanceUnit.CM);
    }

    public double getDistance(DistanceUnit du){
        return distanceSensor.getDistance(du);
    }


    /*
    Touch Methods
     */

    public boolean pressed(){
        touchCache = touchSensor.getState();
        return touchCache;
    }


    /*
    Cache Methods
     */

    public void resetCache(){
        redCache = false;
        blueCache = false;
        touchCache = false;
        redBaseLine = 0;
        blueBaseLine = 0;
    }


    /*
    Telemetry Methods
     */

    public void printTelemetry() {
        telemetry.addData(" ", " ");
        telemetry.addData("Jewel Arm", "isBlue: " + isBlue());
        telemetry.addData("Jewel Arm", "isRed: " + isRed());
        telemetry.addData("Jewel Arm", "pressed: " + pressed());
        telemetry.addData("Jewel Arm", "Distance CM: " + getDistance());
        telemetry.addData("Jewel Arm", "Rotation: " + getCurrentPosition());
    }

}
