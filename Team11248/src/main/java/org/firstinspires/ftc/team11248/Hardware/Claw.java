package org.firstinspires.ftc.team11248.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Tony_Air on 11/7/17.
 */

public class Claw {

    public enum Position {
        OPEN,
        RELEASE,
        GRAB,
    }

    public Position topState;
    public Position bottomState;
    public String header;
    private double[] open, grab, release;
    private int lastRotation = 0;

    private Servo tl, tr, bl, br;
    private DcMotor motor;
    private Telemetry telemetry;


    /**
     * @param hardwareMap robot's hardwaremap
     * @param motorServoNames String array of 4 Servonames of one claw in order of {motor, top_left, top_right, bottom_left, bottom_right}
     * @param open Double array of 4 open values of the servos {top_left, top_right, bottom_left, bottom_right}
     * @param grab Double array of 4 grabbing values of the servos {top_left, top_right, bottom_left, bottom_right}
     */
    public Claw(String header, String[] motorServoNames, double[] open, double[] release, double[] grab, HardwareMap hardwareMap, Telemetry telemetry){

        this.header = header;

        this.open = open;
        this.release = release;
        this.grab = grab;

        this.motor = hardwareMap.dcMotor.get(motorServoNames[0]);
        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.tl = hardwareMap.servo.get(motorServoNames[1]);
        this.tr = hardwareMap.servo.get(motorServoNames[2]);
        this.bl = hardwareMap.servo.get(motorServoNames[3]);
        this.br = hardwareMap.servo.get(motorServoNames[4]);

        this.telemetry = telemetry;
    }

    public void init(){
        open();
        setDriftMode(false);
        resetEncoders();
    }


    /*
    Servo Methods
     */

    public void openTop(){

        if(bottomState == Position.OPEN || bottomState == Position.RELEASE) openBottom();

        tl.setPosition(open[0]);
        tr.setPosition(open[1]);

        topState = Position.OPEN;
    }

    public void openBottom(){
        bl.setPosition(open[2]);
        br.setPosition(open[3]);

        bottomState = Position.OPEN;
    }


    public void releaseTop(){

        if(bottomState == Position.GRAB) releaseBottom();

        tl.setPosition(release[0]);
        tr.setPosition(release[1]);

        topState = Position.RELEASE;
    }

    public void releaseBottom(){

        if(topState == Position.OPEN) releaseTop();

        bl.setPosition(release[2]);
        br.setPosition(release[3]);

        bottomState = Position.RELEASE;
    }


    public void grabTop(){
        tl.setPosition(grab[0]);
        tr.setPosition(grab[1]);

        topState = Position.GRAB;
    }

    public void grabBottom(){

        if(topState == Position.OPEN || topState == Position.RELEASE) grabTop();

        bl.setPosition(grab[2]);
        br.setPosition(grab[3]);

        bottomState = Position.GRAB;
    }


    public void open(){
        openTop();
        openBottom();
    }

    public void release(){
        releaseTop();
        releaseBottom();
    }

    public void grab(){
        grabTop();
        grabBottom();
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

    public void resetEncoders() {
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        recordPosition();
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public int getCurrentPosition(){
        recordPosition();
        return lastRotation;
    }

    public int getLastPosition(){
        return lastRotation;
    }


    /*
    Telemetry Methods
     */

    public void printTelemetry(){

        telemetry.addData(" ", " ");
        telemetry.addData(header + "Top Claw", "Top State: "+ topState.toString());
        telemetry.addData(header + "Bottom Claw", "Bottom State: "+ bottomState.toString());
        telemetry.addData(header + " Claw", "Rotation: " + getCurrentPosition());
    }

}
