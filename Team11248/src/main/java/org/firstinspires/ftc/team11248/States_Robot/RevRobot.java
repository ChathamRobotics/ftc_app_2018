package org.firstinspires.ftc.team11248.States_Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team11248.Hardware.Claw;
import org.firstinspires.ftc.team11248.Hardware.HolonomicDriver_11248;
import org.firstinspires.ftc.team11248.Hardware.Relic_Arm;
import org.firstinspires.ftc.team11248.Hardware.Vuforia_V2;

public class RevRobot extends HolonomicDriver_11248 {

    public Vuforia_V2 vuforia; //TODO: Vuforia implamentations
    public static final String vuforiaKey = "AeTwV0H/////AAAAGfe7ayWmjE9+nI9k65aoO+NQIIujZBIX8AxeoVDf9bwLLNvQ6QwvM+Clc3CE/8Pumv5guDuXMxkERpyJTzSb50PcrH9y/lJC9Zfh0FlPVkkvDnZVNsPEIEsg0Ta5oDlz1jIZmSB/Oxu2qRAyo4jXIsWSmDMdQdpNrwkyKbLfl/CT7PWe23RAdF8oQf5XqnSbKoapQali8MH4+HPOR8r13/k+cZv9eKqUvknmxZPiyJbp4oFzqrWDJSUqwTGQLEdbp76Hjrkuxu3Pa/I4jQSt3RRRbAUrZeV1Z79cLKg+22SvrhUKKzwxeEMcgp4rQzrMXhTL+wE+6sBczuguHmPtWA5w/NsUlevRaLbEionbyXYN";

    BNO055IMU imu;//TODO: gyro implementation
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();


    /*
     * CLAW
     */

    public Claw frontClaw, backClaw;

    private final String[] frontClawNames = {"frontLift", "servo8", "servo10", "servo9", "servo7"};
    private final String[] backClawNames = {"backLift", "servo10", "servo3", "servo11", "servo4"};

    private final double[] close = {0, 0, 0, 0};
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

    public DcMotor jewel;

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

        this.frontClaw = new Claw(frontClawNames, open, release, grab, close, hardwareMap, telemetry);
        this.backClaw = new Claw(backClawNames, open, release, grab, close, hardwareMap, telemetry);

        /*
        RELIC
         */

        relicArm = new Relic_Arm("relic", "servo12", "servo11", pivotVal, grabVal, hardwareMap, telemetry);

        /*
        JEWEL
         */

        jewel = hardwareMap.dcMotor.get("jewel");


        setParameters();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


    }


    public void init(){
        frontClaw.open();
        backClaw.open();
        relicArm.release();
    }

    public void printEncoderVals(){

    }

    public void setParameters(){
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
    }
}
