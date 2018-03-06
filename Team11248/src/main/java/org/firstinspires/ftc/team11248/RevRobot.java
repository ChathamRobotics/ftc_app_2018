package org.firstinspires.ftc.team11248;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.team11248.Hardware.Claw;
import org.firstinspires.ftc.team11248.Hardware.HolonomicDriver_11248;
import org.firstinspires.ftc.team11248.Hardware.Jewel_Arm;
import org.firstinspires.ftc.team11248.Hardware.Relic_Arm;
import org.firstinspires.ftc.team11248.Hardware.Vuforia_V2;

public class RevRobot extends HolonomicDriver_11248 {

    private final double GYRO_THRESHOLD = 5;
    public double[] IMUBaseline = {0, 0, 0};


    /*
    CLAW
     */

    public Claw frontClaw, backClaw;

    private final String[] frontClawNames = {"frontLift", "servo9", "servo7", "servo8", "servo10"};
    private final String[] backClawNames = {"backLift", "servo3", "servo2", "servo4", "servo1"};


//    private final double[] frontOpen = {.5, .30, .65, .275};
//    private final double[] frontRelease = {.45, .55, .6, .5};
//    private final double[] frontGrab = {.2, .625, .3, .65};

    private final double[] frontOpen = {.55, .3, .675, .275};
    private final double[] frontRelease = {.35, .5, .425, .525};
    private final double[] frontGrab = {.2, .675, .275, .7};


    private final double[] backOpen = {.60, .325, .725, .265};
    private final double[] backRelease = {.38, .525, .5, .5};
    private final double[] backGrab = {.25, .675, .35, .65};


    /*
    RELIC
     */

    public Relic_Arm relicArm;
    private final double[] pivotVal = {1, 0, 0};
    private final double[] grabVal = {1, .445};


    /*
    JEWEL
     */

    public Jewel_Arm jewelArm;


    /*
    IMU
     */

    BNO055IMU imu;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    Orientation lastAngles;


    /*
    VUFORIA
     */

    public Vuforia_V2 vuforia;


    /*
    Telemetry
     */

    public Telemetry telemetry;

    public RevRobot(HardwareMap hardwareMap, Telemetry telemetry){

        /*
        DRIVE
         */
        super( hardwareMap.dcMotor.get("FrontLeft"),
                hardwareMap.dcMotor.get("FrontRight"),
                hardwareMap.dcMotor.get("BackLeft"),
                hardwareMap.dcMotor.get("BackRight"),
                telemetry);

        /*
        CLAW
         */

        this.frontClaw = new Claw("Front", frontClawNames, frontOpen, frontRelease, frontGrab, hardwareMap, telemetry);
        this.backClaw = new Claw("Back", backClawNames, backOpen, backRelease, backGrab, hardwareMap, telemetry);


        /*
        RELIC
         */

        relicArm = new Relic_Arm("relic", "servo12", "servo11", pivotVal, grabVal, hardwareMap, telemetry);


        /*
        JEWEL
         */

        jewelArm = new Jewel_Arm("jewel", "color", "touch", hardwareMap, telemetry);


        /*
        IMU
         */

        imu = hardwareMap.get(BNO055IMU.class, "imu");


        /*
        VUFORIA
         */
        vuforia = new Vuforia_V2(hardwareMap);


        /*
        Telemetry
         */

        this.telemetry = telemetry;

    }


    public void init(){
        frontClaw.init();
        backClaw.init();
        relicArm.init();
        jewelArm.init();
        super.initDrive();
    }


    /*
    Telemetry
     */

    public void printIMUTelemetry(){
        telemetry.addData("", "" );
        telemetry.addData("IMU", "Heading: "  + getCurrentHeading());
        telemetry.addData("IMU", "Heading: "  + getCurrentHeading());
        telemetry.addData("IMU", "Heading: "  + getCurrentHeading());

    }

    public void printSensorTelemetry(){
        printIMUTelemetry();
        frontClaw.printTelemetry();
        backClaw.printTelemetry();
        relicArm.printTelemetry();
        jewelArm.printTelemetry();
        telemetry.update();
    }

    public void printTelemetry(){
        frontClaw.printTelemetry();
        backClaw.printTelemetry();
        relicArm.printTelemetry();
        jewelArm.printTelemetry();
        super.printDriveTelemetry();
        telemetry.update();
    }

    public void printCompTelemetry(){
        super.printDriveCompTelemetry();
        telemetry.update();
    }

    public void printAutoTelemetry() { //No telemetry.update(); --- handled in opmode
        printIMUTelemetry();
        frontClaw.printTelemetry();
        backClaw.printTelemetry();
        jewelArm.printTelemetry();
    }


    /*
    Encoders
     */

    public void resetEncoders(){
        frontClaw.resetEncoders();
        backClaw.resetEncoders();
        relicArm.resetEncoders();
        jewelArm.resetEncoders();
        super.resetDriveEncoders();
    }

    public void recordPosition(){
        frontClaw.recordPosition();
        backClaw.recordPosition();
        relicArm.recordPosition();
        jewelArm.recordPosition();
        super.recordDrivePosition();
    }


    /*
    IMU
     */

    public void init_IMU(){
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);
    }

    private void recordAngles(){
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
    }

    public double getCurrentPitch(){
        recordAngles();
        return lastAngles.firstAngle < 0 ? 360 + lastAngles.firstAngle: lastAngles.firstAngle;
    }

    public double getCurrentRoll(){
        recordAngles();
        return lastAngles.firstAngle < 0 ? 360 + lastAngles.firstAngle: lastAngles.firstAngle;
    }

    public double getCurrentHeading(){
        recordAngles();
        return lastAngles.firstAngle < 0 ? 360 + lastAngles.firstAngle: lastAngles.firstAngle;
    }

    public void setIMUBaseline(){
        recordAngles();

        IMUBaseline[0] = getCurrentPitch();
        IMUBaseline[1] = getCurrentRoll();
        IMUBaseline[2] = getCurrentHeading();
    }

    public boolean driveWithGyro(double x, double y, double targetAngle, boolean smooth) { //TODO speed up

        targetAngle %= 360;

        boolean atAngle = false;
        double currentAngle = getCurrentHeading();
        double net = currentAngle - targetAngle; //finds distance to target angle
        double rotation;

        if (Math.abs(net) > 180) { // if shortest path passes 0
            if (currentAngle > 180) //if going counterclockwise
                net = (currentAngle - 360) - targetAngle;

            else //if going clockwise
                net = (360 - targetAngle) + currentAngle;
        }

        // slows down as approaches angle with min threshold of .05
        // each degree adds/subtracts .95/180 values of speed
        rotation = Math.abs(net) * .7 / 180 + .3;

        if (net < 0) rotation *= -1; //if going clockwise, set rotation clockwise (-)

        if (!(Math.abs(net) > GYRO_THRESHOLD)){
            atAngle = true;
            rotation = 0;
        }

        driveWithFixedAngle(x, y, rotation, 360 - getCurrentHeading() + targetAngle, smooth); //Drive with gyros rotation

        telemetry.addData("driveWithGyro", "Heading: " + getCurrentHeading());
        telemetry.addData("driveWithGyro", "Net: " + net);
        telemetry.addData("driveWithGyro", "Speed: " + rotation);
        telemetry.addData("driveWithGyro", "Target: " + targetAngle);
        telemetry.addData("driveWithGyro", "atAngle: " + atAngle);


        return atAngle;
    }

    public boolean driveWithGyro(double x, double y, double targetAngle) {
        return driveWithGyro(x, y, targetAngle, false);
    }


    /**
     *
     * @param x - x direction power
     * @param y - y direction power
     * @param rotate - power for rotation
     * @param fixedAngle -  angle orentation is fixed on - set = to 359 - getGyroAngle()
     */

    public void driveWithFixedAngle(double x, double y, double rotate, double fixedAngle, boolean smooth){

        setOffsetAngle((Math.toRadians(fixedAngle)));
        drive(x, y, rotate, smooth);
    }

    public void driveWithFixedAngle(double x, double y, double rotate, double fixedAngle) {
        driveWithFixedAngle(x, y, rotate, fixedAngle, false);
    }

    public boolean moveToAngle(double targetAngle){
        return driveWithGyro(0,0, targetAngle);
    }

    public boolean isAtAngle(char axis, double targetAngle, double threshold) {

        double currentAngle = 0;

        switch (axis) {
            case 'X':
                currentAngle = getCurrentPitch();
                break;

            case 'Y':
                currentAngle = getCurrentRoll();
                break;

            case 'Z':
                currentAngle = getCurrentHeading();
                break;
        }

        return currentAngle <= (targetAngle + threshold) || (currentAngle - threshold) >= targetAngle;
    }

    public boolean isAtAngle(char axis, double targetAngle) {
        return isAtAngle(axis, targetAngle, GYRO_THRESHOLD);
    }


}
