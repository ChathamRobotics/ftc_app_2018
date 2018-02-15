package org.firstinspires.ftc.team11248.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
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
        CLOSE,
    }

    public Position state;

    private double[] open, grab, close, release;

    private Servo tl, tr, bl, br;

    private DcMotor motor;


    /**
     * @param hardwareMap robot's hardwaremap
     * @param motorServoNames String array of 4 Servonames of one claw in order of {motor, top_left, top_right, bottom_left, bottom_right}
     * @param open Double array of 4 open values of the servos {top_left, top_right, bottom_left, bottom_right}
     * @param grab Double array of 4 grabbing values of the servos {top_left, top_right, bottom_left, bottom_right}
     * @param close Double array of 4 closed values of the servos {top_left, top_right, bottom_left, bottom_right}
     */
    public Claw(String[] motorServoNames, double[] open, double[] release, double[] grab, double[] close, HardwareMap hardwareMap, Telemetry telemetry){

        this.open = open;
        this.release = release;
        this.grab = grab;
        this.close = close;

        this.motor = hardwareMap.dcMotor.get(motorServoNames[0]);

        this.tl = hardwareMap.servo.get(motorServoNames[1]);
        this.tr = hardwareMap.servo.get(motorServoNames[2]);
        this.bl = hardwareMap.servo.get(motorServoNames[3]);
        this.br = hardwareMap.servo.get(motorServoNames[4]);

    }

    public void grabTop(){
        tl.setPosition(grab[0]);
        tr.setPosition(grab[1]);
    }

    public void open(){
        tl.setPosition(open[0]);
        tr.setPosition(open[1]);
        bl.setPosition(open[2]);
        br.setPosition(open[3]);

        state = Position.OPEN;
    }

    public void release(){

        tl.setPosition(release[0]);
        tr.setPosition(release[1]);
        bl.setPosition(release[2]);
        br.setPosition(release[3]);

        state = Position.RELEASE;

    }

    public void grab(){
        tl.setPosition(grab[0]);
        tr.setPosition(grab[1]);
        bl.setPosition(grab[2]);
        br.setPosition(grab[3]);

        state = Position.GRAB;
    }

    public void close() {

        tl.setPosition(close[0]);
        tr.setPosition(close[1]);
        bl.setPosition(close[2]);
        br.setPosition(close[3]);

        state = Position.CLOSE;
    }

    public void setPower(double power){
        motor.setPower(power);
    }

    public void printTelemetry(){
        //TODO: printTelemetry()
    }

    //TODO: record encoder values and movement implementations

}
