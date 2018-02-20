package org.firstinspires.ftc.team11248.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Driving with omni wheels
 */
public class HolonomicDriver_11248 {

    /*
    Constants
     */

    public static final double HOLONOMIC_CORRECTION_ANGLE = Math.PI/4;

    public static final double FRONT_OFFSET = 0;
    public static final double LEFT_OFFSET = Math.PI/2;
    public static final double BACK_OFFSET = Math.PI;
    public static final double RIGHT_OFFSET = 3 * Math.PI / 2;

    public static final double MAX_TURN = .7;
    public static final double MAX_SPEED = .3;
    public static final double SLOW_SPEED = .4;


    /*
    Mode Trackers
     */

    private boolean isSlow = false;
    private boolean isDrift = false;
    private boolean fastMode = false;


    /*
    Hardware
     */

    private Telemetry telemetry;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private int frontLeftRotations, frontRightRotations, backLeftRotations, backRightRotations;


    /*
    Drive Values
     */

    private double FL, FR, BL, BR;
    private double x, y, rot;
    private double angle, radius;
    private double offsetAngle;



    /**
     * creates new HolonomicDriver_11248.
     * @param frontLeft {DcMotor} - front left motor
     * @param frontRight {DcMotor} - front right motor
     * @param backLeft {DcMotor} - back left motor
     * @param backRight {DcMotor} - back right motor
     * @param telemetry {Telemetry} - telemetry
     */

    public HolonomicDriver_11248(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft,
                                 DcMotor backRight, Telemetry telemetry) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.telemetry = telemetry;

        this.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void initDrive(){
        resetDriveEncoders();
        setDriftMode(false);
    }



    /*
    Drive Methods
     */

    public void setMotorPower(double fl, double fr, double bl, double br){
        frontLeft.setPower( Range.clip(fl, -1, 1));
        frontRight.setPower( Range.clip(fr, -1, 1));
        backLeft.setPower( Range.clip(bl, -1, 1));
        backRight.setPower( Range.clip(br, -1, 1));
    }

    public void stop(){
        setMotorPower(0,0,0,0);
    }

    /**
     * Team 11248's driving method for holonomic wheel drive
     * @param xi - values of x to drive (double -1 to +1)
     * @param yi - values of y to drive (double -1 to +1)
     * @param rotate - values for rotation to drive (double -1 to +1)
     * @param smooth - boolean declaring if driving values are smoothed (low values easier to control)
     */

    public void drive(double xi, double yi, double rotate, boolean smooth) {


        /*
        CARTESIAN VALUES
         */

        this.x = Range.clip(xi, -1, 1);
        this.y = Range.clip(yi, -1, 1);
        this.rot = Range.clip(rotate, -1, 1);

        /* This makes the rotation and speed ratios relative to the rotation value
         * So when we don't have a rotation we can drive at full speed instead of a fraction of speed
         * If rotation is being used, a ratio is induced to prevent a value greater than 1
         */

        double MAX_TURN, MAX_SPEED;

        if (smooth) {
            MAX_TURN = Math.abs(rotate) * HolonomicDriver_11248.MAX_TURN;
            MAX_SPEED = 1 - MAX_TURN;

        } else {
            MAX_SPEED = HolonomicDriver_11248.MAX_SPEED;
            MAX_TURN = HolonomicDriver_11248.MAX_TURN;
        }



        /*
        POLAR COORDINATES
         */

        angle = Math.atan2(y, x);
        angle += (HOLONOMIC_CORRECTION_ANGLE) + offsetAngle; //take our angle and shift it 90 deg (PI/4)
        radius = Range.clip( Math.sqrt( (x * x) + (y * y) ), 0, 1);

//        if (smooth || isSlow) radius = radius * radius; //Using a function on variable r will smooth out the slow values but still give full range



        /*
        NEW CARTESIAN VALUES
         */

        /* Takes new angle and radius and converts them into the motor values
         * Multiples by our speed reduction ratio and our slow speed ratio
         */
        double SLOW_SPEED_MULTIPLIER = isSlow ? SLOW_SPEED : 1;
        double FAST_SPEED_MULTIPLIER = fastMode ? Math.sqrt(2) : 1;

        FL = BR = Math.sin(angle) * MAX_SPEED * radius * FAST_SPEED_MULTIPLIER * SLOW_SPEED_MULTIPLIER;
        FR = BL = Math.cos(angle) * MAX_SPEED * radius * FAST_SPEED_MULTIPLIER * SLOW_SPEED_MULTIPLIER;



        /*
        ROTATION
         */

        rotate *= MAX_TURN;

        FL += rotate;
        FR += rotate;
        BL += rotate;
        BR += rotate;


        setMotorPower(FL, FR, BL, BR);
        recordDrivePosition();

    }

    public void drive(double x, double y, double rotate){
        this.drive(x, y, rotate, false);
    }

    public void setDriveMode(DcMotor.RunMode runMode){
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
    }




    /*
    Encoder Methods
     */

    public void recordDrivePosition(){

        frontLeftRotations = frontLeft.getCurrentPosition();
        frontRightRotations = frontRight.getCurrentPosition();
        backLeftRotations = backLeft.getCurrentPosition();
        backRightRotations = backRight.getCurrentPosition();
    }

    public void resetDriveEncoders(){
       setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       recordDrivePosition();
       setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    /*
    Mode Setters
     */

    public void setSlowMode(boolean on) {
        this.isSlow = on;
    }

    public void setDriftMode(boolean on){
        isDrift = on;

        if (isDrift) {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        } else {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

    }

    public void setFastMode(boolean on){
        fastMode = on;
    }


    /*
    Mode Getters
     */

    public boolean isSlow() {
        return isSlow;
    }

    public boolean isDrift(){
        return isDrift;
    }

    public boolean isFast(){
        return fastMode;
    }


    /*
    Mode Toggles
     */

    public void toggleSlowMode() {
        isSlow = !isSlow;
    }

    public void toggleDriftMode(){
        isDrift = !isDrift;
        setDriftMode(isDrift);
    }

    public void toggleFastMode(){
        fastMode = !fastMode;
    }


    /*
    Offset Angle
     */

    public void setOffsetAngle(double angle) {
        offsetAngle = angle;
    }

    public double getOffsetAngle() {
        return offsetAngle;
    }


    /*
    Telemetry Methods
     */

    public void printDriveModes(){
        telemetry.addData(" ", " ");
        telemetry.addData("Holo Driver", "isSlow: " + isSlow());
        telemetry.addData("Holo Driver", "isDrift: " + isDrift());
        telemetry.addData("Holo Driver", "isFast: " + isFast());
    }

    public void printDriveMotorPower(){
        telemetry.addData(" ", " ");
        telemetry.addData("Holo Driver", "FL Power: " + FL);
        telemetry.addData("Holo Driver", "FR Power: " + FR);
        telemetry.addData("Holo Driver", "BL Power: " + BL);
        telemetry.addData("Holo Driver", "BR Power: " + BR);
    }

    public void printDriveXYRVals() {
        telemetry.addData(" ", " ");
        telemetry.addData("Holo Driver", "X: " + x);
        telemetry.addData("Holo Driver", "Y: " + y);
        telemetry.addData("Holo Driver", "Rotation: " + rot);
    }

    public void printDrivePolarVals() {
        telemetry.addData(" ", " ");
        telemetry.addData("Holo Driver", "Angle: " + angle);
        telemetry.addData("Holo Driver", "Ofset Angle: " + offsetAngle);
        telemetry.addData("Holo Driver", "Radius: " + radius);
    }

    public void printDriveRotations(){
        telemetry.addData(" ", " ");
        telemetry.addData("Holo Driver", "FL Rot: " + frontLeftRotations);
        telemetry.addData("Holo Driver", "FR Rot: " + frontRightRotations);
        telemetry.addData("Holo Driver", "BL Rot: " + backLeftRotations);
        telemetry.addData("Holo Driver", "BR Rot: " + backRightRotations);
    }

    public void printDriveTelemetry(){
        telemetry.addData(" ", " ");
        printDriveModes();
        printDriveXYRVals();
        printDrivePolarVals();
        printDriveRotations();
        printDriveMotorPower();
    }
}
