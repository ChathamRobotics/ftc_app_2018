package org.firstinspires.ftc.team11248.Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.NaiveAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by tonytesoriero on 3/6/18.
 */

public class IMU {

    public static final double GYRO_THRESHOLD = 5;
    private final int refreshRate = 500;

    private Telemetry telemetry;
    private BNO055IMU imu1, imu2;

    private double imu1Last, imu2Last, delta_imu1, delta_imu2;

    private BNO055IMU.Parameters parameters, parameters2;

    private double[] lastAngles1 = {0, 0, 0};
    private double[] lastAngles2 = {0, 0, 0};

    private double[] baseLine1 = {0, 0, 0};
    private double[] baseLine2 = {0, 0, 0};


    public IMU(String deviceName, String deviceName2, HardwareMap hardwareMap, Telemetry telemetry){

        this.imu1 = hardwareMap.get(BNO055IMU.class, deviceName);
        this.imu2 = hardwareMap.get(BNO055IMU.class, deviceName2);
        this.telemetry = telemetry;

        this.parameters = new BNO055IMU.Parameters();
        this.parameters2 = new BNO055IMU.Parameters();


    }


    /*
    Inits
     */

    public void init(){


        /*
        Init IMU 1
         */

        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu1.initialize(parameters);
        imu1.startAccelerationIntegration(new Position(), new Velocity(), refreshRate);


         /*
        Init IMU 2
         */

        parameters2.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration2.json"; // see the calibration sample opmode
        parameters2.loggingEnabled      = true;
        parameters2.loggingTag          = "IMU2";

        imu2.initialize(parameters2);
        imu2.startAccelerationIntegration(new Position(), new Velocity(), refreshRate);


        /*
        Record how much IMU 2 is off from IMU 1
         */

        setBaseline();


    }

    public void stop(){
        imu1.stopAccelerationIntegration();
        imu2.stopAccelerationIntegration();
    }

    public void update(){
        Orientation lastAngles1 = imu1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        Orientation lastAngles2 = imu2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);

        this.lastAngles1[0] = (lastAngles1.firstAngle + 180) - baseLine1[0]; //heading
        this.lastAngles1[1] = (lastAngles1.secondAngle + 180) - baseLine1[1]; //roll
        this.lastAngles1[2] = (lastAngles1.thirdAngle + 180) - baseLine1[2]; //pitch

        this.lastAngles2[0] = (lastAngles2.firstAngle + 180) - baseLine2[0]; //heading
        this.lastAngles2[1] = (lastAngles2.secondAngle + 180) - baseLine2[1]; //roll
        this.lastAngles2[2] = (lastAngles2.thirdAngle + 180) - baseLine2[2]; //pitch

    }

    public void logHeadingChanges(){

        //Save last angle
        imu1Last = lastAngles1[0];
        imu2Last = lastAngles2[0];

        //log new angles
        update();

        //find change in both
        delta_imu1 = Math.abs(Math.abs(lastAngles1[0]) - Math.abs(imu1Last));
        delta_imu2 = Math.abs(Math.abs(lastAngles2[0]) - Math.abs(imu2Last));

    }


    /*
   Baseline
    */

    public void setBaseline() {

        for (int i = 0; i<3; i++){ //reset baseline
            baseLine1[i] = 0;
            baseLine2[i] = 0;
        }

        update();

        for (int i = 0; i<3; i++){ //use current angles as baseline
            baseLine1[i] = lastAngles1[i];
            baseLine2[i] = lastAngles2[i];
        }
    }


    /*
    IMU Specific returns
     */



    /*
    Angle Returns
     */
    public double getCurrentPitch(){//Z
        update();
        return lastAngles1[2];
    }

    public double getCurrentRoll(){//X
        update();
        return lastAngles1[1];
    }

    public double getCurrentHeading(){//Y
        logHeadingChanges();
        return (delta_imu1 > delta_imu2) ? lastAngles2[0] : lastAngles1[0];
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


    /*
    Telemetry
     */

    public void printTelemetry(){
        telemetry.addData("", "" );
        telemetry.addData("IMU", "Heading: "  + getCurrentHeading());
        telemetry.addData("IMU", "Roll: "  + getCurrentRoll());
        telemetry.addData("IMU", "Pitch: "  + getCurrentPitch());
        telemetry.addData("IMU", "Delta IMU1: "  + delta_imu1);
        telemetry.addData("IMU", "Delta IMU2: "  + delta_imu2);
    }

}
