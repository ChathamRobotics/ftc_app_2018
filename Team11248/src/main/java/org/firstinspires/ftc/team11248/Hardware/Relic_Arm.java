package org.firstinspires.ftc.team11248.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Relic_Arm {

    public enum Position {
        UP,
        GRAB,
        RELEASE
    }

    public Position state;

    private DcMotor motor;
    private Servo pivot, grab;
    private double up, down, release;
    private double open, closed;

    private Telemetry telemetry;

    /**
     *
     * @param motor
     * @param pivotServo
     * @param grabServo
     * @param pivotVal - [up, down, release]
     * @param grabVal - [open, closed]
     */
    public Relic_Arm(String motor, String pivotServo, String grabServo, double[] pivotVal, double[] grabVal, HardwareMap hardwareMap, Telemetry telemetry){

        this.motor = hardwareMap.dcMotor.get(motor);
        this.pivot = hardwareMap.servo.get(pivotServo);
        this.grab = hardwareMap.servo.get(grabServo);

        up = pivotVal[0];
        down = pivotVal[1];
        release = pivotVal[2];

        open = grabVal[0];
        closed = grabVal[1];

        this.telemetry = telemetry;
    }

    public void setPower(double power){
        motor.setPower(power);
    }

    public void grab(){
        pivot.setPosition(down);
        grab.setPosition(closed);

        state = Position.GRAB;
    }

    public void release(){
        pivot.setPosition(release);
        grab.setPosition(open);

        state = Position.RELEASE;
    }

    public void up(){
        pivot.setPosition(up);
        grab.setPosition(closed);

        state = Position.RELEASE;
    }

    public void printTelemetry(){
        //Todo: log telemetry
    }
}
