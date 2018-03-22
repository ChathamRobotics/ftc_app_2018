package org.firstinspires.ftc.team11248;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
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
import org.firstinspires.ftc.team11248.Hardware.IMU;
import org.firstinspires.ftc.team11248.Hardware.Jewel_Arm;
import org.firstinspires.ftc.team11248.Hardware.Relic_Arm;
import org.firstinspires.ftc.team11248.Hardware.Vuforia_V2;

public class RevRobot extends HolonomicDriver_11248 {

    private double delta_rot =  0;
    private double lastRot =  0;

    /*
    CLAW
     */

    public Claw frontClaw, backClaw;

    private final String[] frontClawNames = {"frontLift", "servo9", "servo7", "servo8", "servo10"};
    private final String[] backClawNames = {"backLift", "servo6", "servo2", "servo4", "servo5"};

    private final double[] frontOpen = {.5375, .3, .685, .275};
    private final double[] frontRelease = {.375, .475, .45, .5};
    private final double[] frontGrab = {.2, .675, .275, .7};

    private final double[] backOpen = {.6, .31, .735, .245};
    private final double[] backRelease = {.4, .5, .525, .475};
    private final double[] backGrab = {.25, .675, .35, .65};


    /*
    RELIC
     */

    public Relic_Arm relicArm;
    private final double[] pivotVal = {1, 0, 0};
    private final double[] grabVal = {1, .425};


    /*
    JEWEL
     */

    public Jewel_Arm jewelArm;


    /*
    IMU
     */

    public IMU imu;


    /*
    VUFORIA
     */

    public Vuforia_V2 vuforia;
    public JewelDetector jewelDetector;



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

        this.frontClaw = new Claw(Claw.Side.FRONT, frontClawNames, frontOpen, frontRelease, frontGrab, hardwareMap, telemetry);
        this.backClaw = new Claw(Claw.Side.BACK, backClawNames, backOpen, backRelease, backGrab, hardwareMap, telemetry);


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

        imu = new IMU("imu", "imu2", hardwareMap, telemetry);

        /*
        VUFORIA
         */
        vuforia = new Vuforia_V2(hardwareMap, telemetry);
        jewelDetector = new JewelDetector();


        /*
        Telemetry
         */

        this.telemetry = telemetry;

    }



    /*
    Init
     */

    public void init(){
        frontClaw.init();
        backClaw.init();
        jewelArm.init();
        imu.init();
        super.initDrive();
        relicArm.init();
    }

    public void safe_init(){
        init();
        relicArm.up();
    }

    public void init_teleop(){
        frontClaw.init();
        backClaw.init();
        jewelArm.init();
        super.initDrive();
        relicArm.init();
        relicArm.up();
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

    public boolean driveWithGyro(double x, double y, double targetAngle, boolean smooth) { //TODO speed up

        targetAngle %= 360;

        if (targetAngle<0) targetAngle += 360;

        boolean atAngle = false;
        double currentAngle = imu.getCurrentHeading();
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

        if (!(Math.abs(net) > IMU.GYRO_THRESHOLD)){
            atAngle = true;
            rotation = 0;
        }

        delta_rot = rotation - lastRot;

        if(Math.abs(delta_rot) > .65){
            rotation -= delta_rot;
        }

        drive(x, y, rotation, smooth); //Drive with gyros rotation

        lastRot = rotation;


        telemetry.addData("driveWithGyro", "Heading: " + currentAngle);
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



    /*
    Telemetry
     */

    public void printSensorTelemetry(){
        vuforia.printTelemetry();
        imu.printTelemetry();
        frontClaw.printTelemetry();
        backClaw.printTelemetry();
        relicArm.printTelemetry();
        jewelArm.printTelemetry();
        super.printDriveRotations();
        telemetry.update();
    }

    public void printAlignmentTelemetry(){

    }


    public void printTelemetry(){
        frontClaw.printTelemetry();
        backClaw.printTelemetry();
        relicArm.printTelemetry();
        jewelArm.printTelemetry();
        imu.printTelemetry();
        super.printDriveTelemetry();
        telemetry.update();
    }

    public void printTeleOpTelemetry(){
        super.printDriveCompTelemetry();
        telemetry.update();
    }

    public void printAutoTelemetry() { //No telemetry.update(); --- handled in opmode
        telemetry.addData("Jewel Detector", "Last image: " + jewelDetector.getLastOrder());
        vuforia.printAutoTelemetry();
        imu.printTelemetry();
        frontClaw.printTelemetry();
        backClaw.printTelemetry();
        jewelArm.printTelemetry();
    }



}
