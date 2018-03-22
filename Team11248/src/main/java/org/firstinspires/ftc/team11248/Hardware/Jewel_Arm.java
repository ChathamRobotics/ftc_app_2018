package org.firstinspires.ftc.team11248.Hardware;

import android.graphics.Color;

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

    public final int MAX_ENCODER_COUNT = 2400;
    private final int COLOR_THRESHOLD = 50;
    private final int ENCODER_THRESHOLD = 5;

    private int lastRotation = 0;
    public int rotationsToWall;
    public final int BACK_UP_ROTATIONS = 150;


    public boolean redCache = false;
    public boolean blueCache = false;

    public boolean hredCache = false;
    public boolean hblueCache = false;

    public boolean touchCache = false;

    public double blueBaseLine = 0;
    public double redBaseLine = 0;

    private DcMotor motor;
    private ColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    private DigitalChannel touchSensor;
    private Telemetry telemetry;

    float hsvBaseline[] = {0F, 0F, 0F};
    float hsv[] = {0F, 0F, 0F};
    final double SCALE_FACTOR = 255;



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


        Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                (int) (colorSensor.green() * SCALE_FACTOR),
                (int) (colorSensor.blue() * SCALE_FACTOR),
                hsvBaseline);
    }

    public boolean isBlue(){//could compaire base line to current blue

        double delta_blue = Math.abs( colorSensor.blue() - blueBaseLine);
        double delta_red =  Math.abs( colorSensor.red() - redBaseLine);

        blueCache = (delta_blue > (delta_red + COLOR_THRESHOLD));
        return blueCache;
    }

    public boolean isRed(){

        double delta_blue = Math.abs( colorSensor.blue() - blueBaseLine);
        double delta_red =  Math.abs( colorSensor.red() - redBaseLine);

        redCache = ((delta_blue + COLOR_THRESHOLD) < delta_red);
        return redCache;
    }

    public boolean isBlueHSV(){
        Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                (int) (colorSensor.green() * SCALE_FACTOR),
                (int) (colorSensor.blue() * SCALE_FACTOR),
                hsv);


        hblueCache = (( hsv[0] + COLOR_THRESHOLD >= 200) || (hsv[0] - COLOR_THRESHOLD <= 200));
        return hblueCache;
    }

    public boolean isRedHSV(){
        Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                (int) (colorSensor.green() * SCALE_FACTOR),
                (int) (colorSensor.blue() * SCALE_FACTOR),
                hsv);

        hblueCache = ( ( hsv[0] + COLOR_THRESHOLD >= 360) || (hsv[0] - COLOR_THRESHOLD <= 360));
        return hredCache;
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

    public void updateCache(){
        isBlue();
        isRed();
        isBlueHSV();
        isRed();
        pressed();
    }

    public void resetCache(){
        redCache = false;
        blueCache = false;
        touchCache = false;
        hredCache =false;
        hblueCache = false;
        float hsvBaseline[] = {0F, 0F, 0F};
        float hsv[] = {0F, 0F, 0F};
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
        telemetry.addData("Jewel Arm", "isHBlue: " + isBlueHSV());
        telemetry.addData("Jewel Arm", "isHRed: " + isRedHSV());
        telemetry.addData("Jewel Arm", "pressed: " + pressed());
        telemetry.addData("Jewel Arm", "Distance CM: " + getDistance());
        telemetry.addData("Jewel Arm", "Rotation: " + getCurrentPosition());
    }

}
