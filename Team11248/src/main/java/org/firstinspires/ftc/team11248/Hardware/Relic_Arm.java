package org.firstinspires.ftc.team11248.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    private int lastRotation = 0;

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
        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.pivot = hardwareMap.servo.get(pivotServo);
        this.grab = hardwareMap.servo.get(grabServo);

        up = pivotVal[0];
        down = pivotVal[1];
        release = pivotVal[2];

        open = grabVal[0];
        closed = grabVal[1];

        this.telemetry = telemetry;
    }

    public void init(){
        grab();
        setDriftMode(false);
        resetEncoders();
    }


    /*
    Servo Methods
     */

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

        state = Position.UP;
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

    public void printTelemetry() {
        telemetry.addData(" ", " ");
        telemetry.addData("Relic Arm", "State: " + state.toString());
        telemetry.addData("Relic Arm", "Rotation: " + getCurrentPosition());
    }
}
