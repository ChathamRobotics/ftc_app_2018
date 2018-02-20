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

    /*
    CLAW
     */

    public Claw frontClaw, backClaw;

    private final String[] frontClawNames = {"frontLift", "servo8", "servo10", "servo9", "servo7"};
    private final String[] backClawNames = {"backLift", "servo10", "servo3", "servo11", "servo4"};

    private final double[] release = {.675, .25, .625, .4};
    private final double[] grab = {.55, .35, .45, .55};
    private final double[] open = {.9, 0, .85, .2};


    /*
    RELIC
     */

    public Relic_Arm relicArm;
    private final double[] pivotVal = {1, 0, 0};
    private final double[] grabVal = {1, .44};


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

        this.frontClaw = new Claw("Front", frontClawNames, open, release, grab, hardwareMap, telemetry);
        this.backClaw = new Claw("Back", backClawNames, open, release, grab, hardwareMap, telemetry);


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

    public void printSensorTelemetry(){
        frontClaw.printTelemetry();
        backClaw.printTelemetry();
        relicArm.printTelemetry();
        jewelArm.printTelemetry();
    }

    public void printTelemetry(){
        printSensorTelemetry();
        super.printDriveTelemetry();
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
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES);
    }

    public double getCurrentPitch(){
        recordAngles();
        return lastAngles.firstAngle;
    }

    public double getCurrentRoll(){
        recordAngles();
        return lastAngles.secondAngle;
    }

    public double getCurrentHeading(){
        recordAngles();
        return lastAngles.thirdAngle;
    }

    public boolean moveToAngle(double heading){ //TODO: add moveToAngle, isAtAngle, isAtHeading/Roll/Pitch


        return true;
    }

}
